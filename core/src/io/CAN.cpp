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
#include <sstream>
#include <cstring>
#include <mutex>
#include <exception>

#include "ossian/io/IO.hpp"

#ifdef __linux__
namespace ossian
{
// CANBus

CANBus::CANBus(CANManager* manager, std::string const& location, bool isLoopback) :
	m_IsOpened(false), m_IsLoopback(isLoopback),
	m_FD(-1), m_Location(location), m_Manager(manager)
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

std::shared_ptr<CANDevice> CANBus::AddDevice(unsigned int id)
{
	const auto it = m_DeviceMap.find(id);
	std::shared_ptr<CANDevice> device;
	if (it == m_DeviceMap.end())
	{
		device = std::make_shared<CANDevice>(this, id);
		m_DeviceMap.insert(std::make_pair(id, device));
		UpdateFilter();
	}
	else
	{
		device = it->second;
	}
	return device;
}

void CANBus::Read() const
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
			it->second->Invoke(length, buffer.get());
		}
	}
}

void CANBus::WriteRaw(const unsigned id, const size_t length, const uint8_t* data) const
{
	std::stringstream ss;
	for (size_t i{}; i < length; ++i)
	{
		ss << fmt::format("{:02x}", data[i]);
		ss << " ";
	}
	spdlog::info("canlen=[{}]: candata={}", length, ss.str());
	if (true == m_IsOpened)
	{
		struct can_frame rawFrame {};
		rawFrame.can_id = id;
		rawFrame.can_dlc = length;
		memcpy(rawFrame.data, data, length);
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

const std::vector<CANDevice*> CANBus::GetDevices()
{
	std::vector<CANDevice*> devices;
	for (auto& it : m_DeviceMap)
	{
		devices.push_back(it.second.get());
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

// CANManager
CANManager::CANManager(IOListener* listener)
{
	AttachListener(listener);
}

CANBus* CANManager::AddBus(std::string const& location, bool isLoopback)
{
	auto bus = std::make_shared<CANBus>(this, location, isLoopback);
	m_Listener->AddBus(bus.get());
	m_BusMap.insert(std::make_pair(location, bus));
	return bus.get();
}

bool CANManager::DelBus(std::string const& location)
{
	auto bus = Bus(location);
	if(bus == nullptr)
	{
		return false;
	}
	m_Listener->DelBus(bus);
	return m_BusMap.erase(location);
}

void CANManager::WriteTo(const BaseDevice* device, const size_t length, const uint8_t* data)
{
	device->WriteRaw(length, data);
}

std::shared_ptr<CANDevice> CANManager::AddDevice(std::string const& location,
                                                 const unsigned int id)
{
	auto bus = Bus(location);
	if (nullptr == bus)
	{
		bus = AddBus(location);
	}
	return bus->AddDevice(id);
}

CANBus* CANManager::Bus(std::string const& location) const
{
	const auto it = m_BusMap.find(location);
	if (it == m_BusMap.end())
	{
		return nullptr;
	}
	return it->second.get();
}

std::vector<CANBus*> CANManager::GetBuses() const
{
	std::vector<CANBus*> buses;
	for (auto& it : m_BusMap)
	{
		buses.push_back(it.second.get());
	}
	return buses;
}

} // ossian
#endif