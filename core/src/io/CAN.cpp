#include "ossian/io/CAN.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
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
#include <mutex>
#include <exception>

#include "ossian/io/IO.hpp"
#include "ossian/io/IOError.hpp"

#ifdef __linux__
namespace ossian
{
// CANBus

CANBus::CANBus(std::shared_ptr<CANManager> manager, std::string location, bool isLoopback) :
	m_IsOpened(false), m_IsLoopback(isLoopback),
	m_FD(-1), m_Location(std::move(location)), m_Manager(manager)
{
	CANBus::Open();
}

CANBus::~CANBus()
{
	try
	{
		CANBus::Close();
	}
	catch (std::exception & err)
	{
		std::abort();
	}
}

bool CANBus::Open()
{
	struct ifreq ifr;
	struct sockaddr_can address; //设置can设备
	int loopback = m_IsLoopback ? 1 : 0;
	m_FD = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW); // 非阻塞模式
	strcpy(ifr.ifr_name, m_Location.c_str()); //指定can设备
	if (ioctl(m_FD, SIOCGIFINDEX, &ifr) < 0)
	{
		m_IsOpened = false;
		return false;
	}
	address.can_family = AF_CAN;
	address.can_ifindex = ifr.ifr_ifindex;
	if (bind(m_FD, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0) // 绑定Socket
	{
		m_IsOpened = false;
		return false;
	}
	if (setsockopt(m_FD, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
	{
		m_IsOpened = false;
		return false;
	}
	UpdateFilter();
	m_IsOpened = true;
	return true;
}

bool CANBus::Close()
{
	close(m_FD); //关闭套接字
	m_IsOpened = false;
	return true;
}

std::shared_ptr<BaseDevice> CANBus::AddDevice(unsigned int id)
{
	const auto it = m_DeviceMap.find(id);
	std::shared_ptr<CANDevice> device;
	if (it == m_DeviceMap.end())
	{
		device = std::make_shared<CANDevice>(shared_from_this(), id);
		m_DeviceMap.insert(std::make_pair(id, device));
		UpdateFilter();
	}
	else
	{
		device = it->second;
	}
	return std::dynamic_pointer_cast<BaseDevice>(device);
}

void CANBus::Read()
{
	if (true == m_IsOpened)
	{
		struct can_frame rawFrame {};
		while (true)
		{
			const auto bytes = read(m_FD, &rawFrame, sizeof(rawFrame));
			if ((bytes < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR))) //这几种错误码都说明还有数据待接收
			{
				continue;//继续接收数据
			}
			break;//跳出接收循环
		}

		const auto id = rawFrame.can_id;
		const size_t length = rawFrame.can_dlc;
		std::shared_ptr<uint8_t[]> buffer(new uint8_t[length]());
		memcpy(buffer.get(), rawFrame.data, length);

		auto it = m_DeviceMap.find(id);
		if (it != m_DeviceMap.end())
		{
			it->second->Invoke(length, buffer);
		}
	}
}

void CANBus::WriteRaw(unsigned int id, size_t length, std::shared_ptr<uint8_t[]> data)
{
	if (true == m_IsOpened)
	{
		struct can_frame rawFrame {};
		rawFrame.can_id = id;
		rawFrame.can_dlc = length;
		memcpy(rawFrame.data, data.get(), length);
		static std::mutex writeMutex;
		{
			std::lock_guard<std::mutex> writeLock(writeMutex);
			while (1)
			{
				const int bytes = write(m_FD, &rawFrame, sizeof(rawFrame));
				if ((bytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待处理
				{
					continue;//继续接收数据
				}
				break;//跳出接收循环
			}
		}
	}
}

std::vector<std::shared_ptr<BaseDevice>> CANBus::GetDevices()
{
	std::vector<std::shared_ptr<BaseDevice>> devices;
	for (auto& it : m_DeviceMap)
	{
		devices.push_back(it.second);
	}
	return devices;
}

void CANBus::UpdateFilter()
{
	const size_t size = m_DeviceMap.size();
	std::vector<struct can_filter> filters;
	for (auto&& it : m_DeviceMap)
	{
		struct can_filter rf;
		rf.can_id = it.first;
		rf.can_mask = (CAN_EFF_FLAG | CAN_SFF_MASK); // Standard frame
		filters.push_back(rf);
	}
	setsockopt(m_FD, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * size); // 设置过滤规则
}

// CANDevice

CANDevice::CANDevice(std::shared_ptr<CANBus> bus, const unsigned int id) noexcept
	: m_Id(id), m_Bus(bus), m_Callback([](std::shared_ptr<BaseDevice>,
									   size_t,
									   std::shared_ptr<uint8_t[]>)
									   {})
{}

// CANManager

const std::shared_ptr<BaseHardwareBus> CANManager::AddBus(std::string const& location)
{
	return AddBus(location, false);
}

const std::shared_ptr<BaseHardwareBus> CANManager::AddBus(std::string const& location, bool isLoopback)
{
	auto bus = std::make_shared<CANBus>(shared_from_this(), location, isLoopback);
	m_BusMap.insert(std::make_pair(location, bus));
	return bus;
}

bool CANManager::DelBus(std::shared_ptr<BaseHardwareBus> bus)
{
	return m_BusMap.erase(bus->Location());
}

bool CANManager::DelBus(std::string const& location)
{
	return m_BusMap.erase(location);
}

void CANManager::WriteTo(std::shared_ptr<BaseDevice> const& device, size_t length, ::std::shared_ptr<unsigned char[]> const& data)
{
	device->WriteRaw(length, data);
}

const std::shared_ptr<BaseDevice> CANManager::AddDevice(std::shared_ptr<CANBus> const& bus,
                                                        const unsigned id)
{
	return bus->AddDevice(id);
}

const std::shared_ptr<BaseDevice> CANManager::AddDevice(std::string const& location,
                                                        const unsigned int id)
{
	auto bus = Bus(location);
	if (nullptr == bus)
	{
		bus = AddBus(location);
	}
	return std::dynamic_pointer_cast<CANBus>(bus)->AddDevice(id);
}

const std::shared_ptr<BaseHardwareBus> CANManager::Bus(std::string const& location) const
{
	const auto it = m_BusMap.find(location);
	if (it == m_BusMap.end())
	{
		return nullptr;
	}
	return it->second;
}

const std::vector<std::shared_ptr<BaseHardwareBus>> CANManager::GetBuses() const
{
	std::vector<std::shared_ptr<BaseHardwareBus>> buses;
	for (auto& it : m_BusMap)
	{
		buses.push_back(it.second);
	}
	return buses;
}

} // ossian
#endif