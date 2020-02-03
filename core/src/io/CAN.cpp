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
#include <tuple>
#include <exception>

#include "ossian/io/IO.hpp"
#include "ossian/io/IOError.hpp"

#ifdef __linux__
namespace ossian
{
	// CANBus

	CANBus::CANBus(std::string location, bool isLoopback) :
		m_IsOpened(false), m_IsLoopback(isLoopback), m_FD(-1), m_Location(std::move(location))
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

	FileDescriptor CANBus::FD() const noexcept
	{
		return m_FD;
	}

	std::string CANBus::Location() const noexcept
	{
		return m_Location;
	}

	bool CANBus::IsOpened() const noexcept
	{
		return m_IsOpened;
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

	std::shared_ptr<BaseDevice> CANBus::AddDevice(unsigned int id, std::function<ReceiveCallback> callback)
	{
		const auto it = m_DeviceMap.find(id);
		std::shared_ptr<CANDevice> device;
		if (it == m_DeviceMap.end())
		{
			device = std::make_shared<CANDevice>(shared_from_this(), id, callback);
			m_DeviceMap.insert(std::make_pair(id, device));
			UpdateFilter();
		}
		else
		{
			device = it->second;
			device->SetCallback(callback);
		}
		return std::dynamic_pointer_cast<BaseDevice>(device);
	}

	void CANBus::Read()
	{
		if (true == m_IsOpened)
		{
			struct can_frame rawFrame {};
			while (1)
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

	CANDevice::CANDevice(std::shared_ptr<CANBus> bus, unsigned int id, std::function<ReceiveCallback> callback) noexcept
		: m_Bus(bus), m_Id(id), m_Callback(callback)
	{
	}

	std::shared_ptr<IIOBus> CANDevice::Bus()
	{
		return std::dynamic_pointer_cast<IIOBus>(m_Bus);
	}

	void CANDevice::Invoke(size_t length, std::shared_ptr<uint8_t[]> data)
	{
		m_Callback(length, data);
	}

	void CANDevice::WriteRaw(size_t length, std::shared_ptr<uint8_t[]> data)
	{
		m_Bus->WriteRaw(m_Id, length, data);
	}

	void CANDevice::SetCallback(std::function<ReceiveCallback> callback)
	{
		m_Callback = callback;
	}

	// CANManager

	IOType CANManager::Type() const noexcept
	{
		return IOType::CAN;
	}

	std::shared_ptr<IIOBus> CANManager::AddBus(const std::string location)
	{
		return AddBus(location, false);
	}

	std::shared_ptr<IIOBus> CANManager::AddBus(std::string location, bool isLoopback)
	{
		auto device = std::make_shared<CANBus>(location, isLoopback);
		m_BusMap.insert(std::make_pair(location, device));
		return device;
	}

	bool CANManager::DelBus(std::shared_ptr<IIOBus> bus)
	{
		return m_BusMap.erase(bus->Location());
	}

	bool CANManager::DelBus(std::string location)
	{
		return m_BusMap.erase(location);
	}

	void CANManager::WriteTo(std::shared_ptr<BaseDevice> device, size_t length, std::shared_ptr<uint8_t[]> data)
	{
		device->WriteRaw(length, data);
	}

	std::shared_ptr<BaseDevice> CANManager::AddDevice(std::shared_ptr<CANBus> bus,
		const unsigned int id,
		const std::function<ReceiveCallback> callback)
	{
		return bus->AddDevice(id, callback);
	}

	std::shared_ptr<IIOBus> CANManager::Bus(const std::string location)
	{
		const auto it = m_BusMap.find(location);
		if (it == m_BusMap.end())
		{
			throw std::runtime_error("No such CAN device");
			return nullptr;
		}
		return it->second;
	}

	std::vector<std::shared_ptr<IIOBus>> CANManager::GetBuses()
	{
		std::vector<std::shared_ptr<IIOBus>> buses;
		for (auto& it : m_BusMap)
		{
			buses.push_back(it.second);
		}
		return buses;
	}

} // ossian
#endif