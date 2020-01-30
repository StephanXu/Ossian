#ifndef OSSIAN_CORE_IO_UART
#define OSSIAN_CORE_IO_UART
#ifdef __linux__
#include <termios.h>
#include <fcntl.h> 
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

#include <functional>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>
#include <cstring>
#include <tuple>
#include <exception>

#include "IO.hpp"
#include "IOError.hpp"


namespace ossian
{
	// 0->CAN ID 1->数据长度 2->数据
	class UART : public IIO
	{
	public:
		const size_t MAX_LENGTH = 2048;
		enum Baudrate
		{
			R0 = 0000000,
			R50 = 0000001,
			R75 = 0000002,
			R110 = 0000003,
			R134 = 0000004,
			R150 = 0000005,
			R200 = 0000006,
			R300 = 0000007,
			R600 = 0000010,
			R1200 = 0000011,
			R1800 = 0000012,
			R2400 = 0000013,
			R4800 = 0000014,
			R9600 = 0000015,
			R19200 = 0000016,
			R38400 = 0000017,
			R57600 = 0010001,
			R115200 = 0010002,
			R230400 = 0010003,
			R460800 = 0010004,
			R500000 = 0010005,
			R576000 = 0010006,
			R921600 = 0010007,
			R1000000 = 0010010,
			R1152000 = 0010011,
			R1500000 = 0010012,
			R2000000 = 0010013,
			R2500000 = 0010014,
			R3000000 = 0010015,
			R3500000 = 0010016,
			R4000000 = 0010017
		};
		enum DataBits
		{
			DataBits5 = 0000000,
			DataBits6 = 0000020,
			DataBits7 = 0000040,
			DataBits8 = 0000060
		};
		enum StopBits
		{
			StopBits1 = 0,
			StopBits2 = 1,
		};
		enum Parity
		{
			ParityNone = 0,
			ParityEven = 1,
			ParityOdd = 2
		};
		enum FlowControl
		{
			FlowControlNone = 0,
			FlowControlHardware = 1,
			FlowControlSoftware = 2
		};
		UART() = delete;
		UART(std::string location, Baudrate baudrate, FlowControl flowctrl, DataBits databits, StopBits stopbits, Parity parity) noexcept :
			m_Baudrate(baudrate), m_FlowCtrl(flowctrl), m_DataBits(databits), m_StopBits(stopbits), m_Parity(parity)
		{
			m_Location = location;
			Open();
		}
		UART(const UART& canDevice) = delete;
		UART(UART&& uart)
		{
			*this = std::move(uart);
		}
		UART& operator=(UART&& uart)
		{
			m_Location = uart.m_Location;
			return *this;
		}
		~UART()
		{
			try
			{
				Close();
			}
			catch (std::exception & err)
			{
				std::abort();
			}
		}
		FileDescriptor FD() const noexcept { return m_FD; }
		std::string Location() const noexcept { return m_Location; }

		bool Open()
		{
			std::string devLocation = "/dev/" + m_Location;
			struct termios opt;
			auto SetFlag = [](tcflag_t& t, tcflag_t bit) {return t |= bit; };
			auto ClearFlag = [](tcflag_t& t, tcflag_t bit) {return t &= ~bit; };
			m_FD = open(devLocation.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
			if (m_FD < 0)
			{
				throw std::runtime_error("Device open failed!");
				return false;
			}
			// 指定波特率
			tcgetattr(m_FD, &opt);
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
			case UART::StopBits1:
				ClearFlag(opt.c_cflag, CSTOPB);
				break;
			case UART::StopBits2:
				SetFlag(opt.c_cflag, CSTOPB);
				break;
			}
			switch (m_Parity)
			{
			case UART::ParityNone:
				ClearFlag(opt.c_cflag, PARENB);
				break;
			case UART::ParityEven:
				SetFlag(opt.c_cflag, PARENB);
				ClearFlag(opt.c_cflag, PARODD);
				break;
			case UART::ParityOdd:
				SetFlag(opt.c_cflag, PARENB);
				SetFlag(opt.c_cflag, PARODD);
				break;
			}
			switch (m_FlowCtrl)
			{
			case UART::FlowControlNone:
				ClearFlag(opt.c_cflag, CRTSCTS);
				break;
			case UART::FlowControlHardware:
				SetFlag(opt.c_cflag, CRTSCTS);
				throw std::runtime_error("Hardware flow control not supported");
				return false;
				break;
			case UART::FlowControlSoftware:
				ClearFlag(opt.c_cflag, CRTSCTS);
				ClearFlag(opt.c_cflag, IXON | IXOFF | IXANY);
				throw std::runtime_error("Software flow control not supported");
				return false;
				break;
			}
			ClearFlag(opt.c_lflag, ICANON | ECHO | ECHOE | ISIG);
			ClearFlag(opt.c_iflag, BRKINT | ICRNL | INPCK | ISTRIP | IXON); // exp
			if (tcsetattr(m_FD, TCSANOW, &opt) < 0)
			{
				throw std::runtime_error("UART Configure failed");
				return false;
			}
			return true;
		}

		bool Close()
		{
			close(m_FD); //关闭套接字
			return true;
		}

		bool AddReceiveCallback(uint32_t id, std::function<ReceiveCallback> callback)
		{
			id = 0; // 对于串口、不存在ID
			auto it = m_InIdMap.find(id);
			if (m_InIdMap.end() != it)
			{
				return false; //Maybe need exception
			}
			m_InIdMap.insert(std::make_pair(id, callback));
			return true;
		}

		bool RemoveReceiveCallback(uint32_t id)
		{
			auto it = m_InIdMap.find(id);
			if (m_InIdMap.end() == it)
			{
				return false;
			}
			m_InIdMap.erase(it);
			return true;
		}

		FrameData ReadRaw()
		{
			int	nbytes;
			char buf[MAX_LENGTH];
			while (1)
			{
				nbytes = read(m_FD, &buf, sizeof(buf));
				if ((nbytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待接收
				{
					continue;
				}
				break;//跳出接收循环
			}
			unsigned int id = 0;
			size_t length = nbytes;
			auto buffer = std::shared_ptr<uint8_t[]>(new uint8_t[length]);
			memcpy(buffer.get(), buf, length);
			return std::move(std::make_tuple(id, length, buffer));
		}

		FrameData Read()
		{
			FrameData frameData = ReadRaw();//raw frame
			auto it = m_InIdMap.find(std::get<0>(frameData));
			if (m_InIdMap.end() != it)
			{
				std::apply(it->second, frameData);
			}
			return frameData;
		}

		void WriteRaw(unsigned int id, size_t length, uint8_t* data)
		{
			while (1)
			{
				int	nbytes = write(m_FD, &data, sizeof(uint8_t) * length);
				if ((nbytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待处理
				{
					continue;
				}
				break;//跳出接收循环
			}
		}
	private:
		Baudrate m_Baudrate;
		FlowControl m_FlowCtrl;
		DataBits m_DataBits;
		StopBits m_StopBits;
		Parity m_Parity;
	};

	class UARTManager : public IIOManager
	{
	public:
		UARTManager()
		{
		}
		IOType Type() const noexcept { return IOType::UART; }
		// 发送原始数据到指定位置
		void SendToRaw(std::string location, unsigned int id, size_t length, uint8_t* data)
		{
			auto dev = FindDevice(location);
			dev->WriteRaw(id, length, data);
		}
		// 从指定位置读取FrameData
		FrameData ReadFrom(std::string location)
		{

			auto dev = FindDevice(location);
			return dev->Read();
		}
		//注册设备
		bool AddDevice(std::string location)
		{
			return AddDevice(location, UART::Baudrate::R115200, UART::FlowControl::FlowControlNone,
				UART::DataBits::DataBits8, UART::StopBits::StopBits1, UART::Parity::ParityNone);
		}
		bool AddDevice(std::string location, UART::Baudrate baudrate, UART::FlowControl flowctrl,
			UART::DataBits databits, UART::StopBits stopbits, UART::Parity parity)
		{
			auto device = std::make_shared<UART>(location, baudrate, flowctrl, databits, stopbits, parity);
			return m_DeviceMap.Insert(device);
		}
		//销毁设备
		bool DelDevice(std::string location)
		{
			return m_DeviceMap.Erase(location);
		}
		bool DelDevice(FileDescriptor fd)
		{
			return m_DeviceMap.Erase(fd);
		}
		//获取设备描述符
		FileDescriptor DeviceFD(std::string location)
		{
			auto dev = FindDevice(location);
			return dev->FD();
		}
		void AddCallback(std::string location, uint32_t id, std::function<ReceiveCallback> callback)
		{
			id = 0;
			auto dev = FindDevice(location);
			dev->AddReceiveCallback(id, callback);
		}
		UART* FindDevice(std::string location)
		{
			auto it = m_DeviceMap.Find(location);
			if (nullptr == it)
			{
				throw std::runtime_error("No such CAN device");
			}
			return it.get();
		}
		UART* FindDevice(FileDescriptor fd)
		{
			auto it = m_DeviceMap.Find(fd);
			if (nullptr == it)
			{
				throw std::runtime_error("No such CAN device");
			}
			return it.get();
		}
		std::vector<FileDescriptor> FDs()
		{
			std::vector<FileDescriptor> fd;
			for (auto it = m_DeviceMap.begin(); it != m_DeviceMap.end(); ++it)
			{
				fd.push_back(it->second->FD());
			}
			return std::move(fd);
		}
	private:
		DeviceMap<UART> m_DeviceMap;
	};
} // ossian

#endif // __linux__
#endif // OSSIAN_CORE_IO_UART