#include "ossian/io/UART.hpp"

#ifdef __linux__
#include <termios.h>
#include <fcntl.h> 
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <mutex>
#include <cstring>

#include <spdlog/spdlog.h>
namespace ossian
{
// UARTBus

UARTBus::UARTBus(UARTManager* manager,
				 std::string const& location,
				 const UARTProperties::Baudrate baudrate,
				 const UARTProperties::FlowControl flowctrl,
				 const UARTProperties::DataBits databits,
				 const UARTProperties::StopBits stopbits,
				 const UARTProperties::Parity parity)
	: m_IsOpened(false), m_FD(-1), m_Location(std::move(location)),
	m_Baudrate(baudrate), m_FlowCtrl(flowctrl), m_DataBits(databits), m_StopBits(stopbits), m_Parity(parity),
	m_Manager(manager)
{
	UARTBus::Open();
}

UARTBus::~UARTBus()
{
	try
	{
		UARTBus::Close();
	}
	catch (std::exception & err)
	{
		std::abort();
	}
}

bool UARTBus::Open()
{
	FileDescriptor fd;
	struct termios opt;
	auto SetFlag = [](tcflag_t& t, tcflag_t bit) { return t |= bit; };
	auto ClearFlag = [](tcflag_t& t, tcflag_t bit) { return t &= ~bit; };
	fd = open(m_Location.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		throw std::runtime_error("Device open failed! Location: "+m_Location);
		return false;
	}
	// 指定波特率
	tcgetattr(fd, &opt);
	opt.c_cc[VMIN] = 0;
	opt.c_cc[VTIME] = 0;
	ClearFlag(opt.c_cflag, CIBAUD);
	cfsetispeed(&opt, m_Baudrate);
	cfsetospeed(&opt, m_Baudrate);
	SetFlag(opt.c_cflag, CLOCAL | CREAD); // 必须开启
	ClearFlag(opt.c_cflag, CSIZE);
	SetFlag(opt.c_cflag, m_DataBits); // 数据位设置
	switch (m_StopBits)
	{
	case UARTProperties::StopBits1:
		ClearFlag(opt.c_cflag, CSTOPB);
		break;
	case UARTProperties::StopBits2:
		SetFlag(opt.c_cflag, CSTOPB);
		break;
	}
	switch (m_Parity)
	{
	case UARTProperties::ParityNone:
		ClearFlag(opt.c_cflag, PARENB);
		break;
	case UARTProperties::ParityEven:
		SetFlag(opt.c_cflag, PARENB);
		ClearFlag(opt.c_cflag, PARODD);
		break;
	case UARTProperties::ParityOdd:
		SetFlag(opt.c_cflag, PARENB);
		SetFlag(opt.c_cflag, PARODD);
		break;
	}
	switch (m_FlowCtrl)
	{
	case UARTProperties::FlowControlNone:
		ClearFlag(opt.c_cflag, CRTSCTS);
		break;
	case UARTProperties::FlowControlHardware:
		SetFlag(opt.c_cflag, CRTSCTS);
		throw std::runtime_error("Hardware flow control not supported");
	case UARTProperties::FlowControlSoftware:
		ClearFlag(opt.c_cflag, CRTSCTS);
		ClearFlag(opt.c_cflag, IXON | IXOFF | IXANY);
		throw std::runtime_error("Software flow control not supported");
	}
	ClearFlag(opt.c_lflag, ICANON | ECHO | ECHOE | ISIG);
	ClearFlag(opt.c_iflag, BRKINT | ICRNL | INPCK | ISTRIP | IXON); // exp
	if (tcsetattr(fd, TCSANOW, &opt) < 0)
	{
		throw std::runtime_error("Device attribute failed to set! Location: " + m_Location);
		m_IsOpened = false;
		return false;
	}
	tcflush(fd, TCIOFLUSH);
	m_FD = fd;
	m_IsOpened = true;
	return true;
}

bool UARTBus::Close()
{
	close(m_FD); //关闭套接字
	m_IsOpened = false;
	return true;
}

std::shared_ptr<UARTDevice> const& UARTBus::AddDevice()
{
	if (nullptr == m_Device)
	{
		m_Device = std::make_shared<UARTDevice>(this);
	}
	return m_Device;
}

void UARTBus::Read() const
{
	if (true == m_IsOpened)
	{
		int	nbytes;
		char buf[MAX_LENGTH];
		while (true)
		{
			nbytes = read(m_FD, &buf, sizeof(buf));
			if ((nbytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待接收
			{
				continue;
			}
			break;//跳出接收循环
		}
		size_t length = nbytes;
		std::shared_ptr<uint8_t[]> buffer(new uint8_t[length]());
		memcpy(buffer.get(), buf, length);
		m_Device->Invoke(length, std::move(buffer));
	}
}

void UARTBus::WriteRaw(size_t length, const uint8_t* data) const
{
	if (true == m_IsOpened)
	{
		static std::mutex writeMutex;
		{
			std::lock_guard<std::mutex> writeLock(writeMutex);
			while (1)
			{
				const int bytes = write(m_FD, data, sizeof(uint8_t) * length);
				if ((bytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待处理
				{
					continue;//继续接收数据
				}
				break;//跳出接收循环
			}
		}
	}
}

std::vector<UARTDevice*> UARTBus::GetDevices() const
{
	std::vector<UARTDevice*> devices;
	devices.push_back(m_Device.get());
	return devices;
}

// UARTManager

UARTManager::UARTManager(IOListener* listener)
{
	AttachListener(listener);
}

/* 使用样例
AddBus(location,
      UARTProperties::Baudrate::R115200,
      UARTProperties::FlowControl::FlowControlNone,
      UARTProperties::DataBits::DataBits8,
      UARTProperties::StopBits::StopBits1,
      UARTProperties::Parity::ParityNone);
	  */

UARTBus* UARTManager::AddBus(std::string const& location,
                             const UARTProperties::Baudrate baudrate,
                             const UARTProperties::FlowControl flowctrl,
                             const UARTProperties::DataBits databits,
                             const UARTProperties::StopBits stopbits,
                             const UARTProperties::Parity parity)
{
	auto bus = std::make_shared<UARTBus>(this, location, baudrate, flowctrl, databits, stopbits, parity);
	m_BusMap.insert(std::make_pair(location, bus));
	m_Listener->AddBus(m_Priority, bus.get());
	return bus.get();
}

bool UARTManager::DelBus(std::string const& location)
{
	auto bus = Bus(location);
	if(bus == nullptr)
	{
		return false;
	}
	m_Listener->DelBus(m_Priority, bus);
	return m_BusMap.erase(location);
}

void UARTManager::WriteTo(const UARTDevice* device, const size_t length, const uint8_t* data)
{
	device->WriteRaw(length, data);
}

std::shared_ptr<UARTDevice> UARTManager::AddDevice(std::string const& location,
												   const UARTProperties::Baudrate baudrate,
												   const UARTProperties::FlowControl flowctrl,
												   const UARTProperties::DataBits databits,
												   const UARTProperties::StopBits stopbits,
												   const UARTProperties::Parity parity)
{
	auto bus = Bus(location);
	if (nullptr == bus)
	{
		bus = AddBus(location, baudrate, flowctrl, databits, stopbits, parity);
	}
	else
	{
		spdlog::warn("Adding duplicate device {}!", location);
	}
	return bus->AddDevice();
}

UARTBus* UARTManager::Bus(std::string const& location) const
{
	const auto it = m_BusMap.find(location);
	if (it == m_BusMap.end())
	{
		return nullptr;
	}
	return it->second.get();
}

std::vector<UARTBus*> UARTManager::GetBuses() const
{
	std::vector<UARTBus*> buses;
	for (auto& it : m_BusMap)
	{
		buses.push_back(it.second.get());
	}
	return buses;
}

std::vector<ossian::UARTDevice*> ossian::UARTManager::GetDevices() const
{
	//[TODO] 改善代码执行效率
	std::vector<ossian::UARTDevice*> devices;
	auto busesMap = m_BusMap;
	for (auto && it : busesMap)
	{
		auto devs = it.second->GetDevices();
		for (auto && dev : devs)
		{
			devices.push_back(dev);
		}
	}
	return devices;
}

} // ossian
#endif