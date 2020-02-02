/**
 * @file	ossian\io\CAN.hpp
 *
 * @brief	Declares the can class
 */
#ifndef OSSIAN_CORE_IO_CAN
#define OSSIAN_CORE_IO_CAN
#ifdef __linux__
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

#include "IO.hpp"
#include "IOError.hpp"

 /**
  * @namespace	ossian
  *
  * @brief	.
  */

namespace ossian
{

	/**
	 * Controller Area Network Identifier structure
	 *
	 * bit 0-28	: CAN identifier (11/29 bit)
	 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
	 * bit 30	: remote transmission request flag (1 = rtr frame)
	 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
	 */
	class CANBus;
	class CANDevice;
	class CANManager;

	class CANDevice : public BaseDevice
	{
	public:
		CANDevice() = delete;
		CANDevice(std::shared_ptr<CANBus> bus,
			unsigned int id,
			std::function<ReceiveCallback> callback) noexcept
			:m_Bus(bus), m_Id(id), m_Callback(callback) {};

		std::shared_ptr<IIOBus> Bus() { return std::dynamic_pointer_cast<IIOBus>(m_Bus); }
		void Invoke(size_t length, std::shared_ptr<uint8_t[]> data)
		{
			m_Callback(length, data);
		}
		void WriteRaw(size_t length, std::shared_ptr<uint8_t[]> data)
		{
			m_Bus->WriteRaw(m_Id, length, data);
		}
		void SetCallback(std::function<ReceiveCallback> callback)
		{
			m_Callback = callback;
		} //非线程安全
	private:
		unsigned int m_Id;
		std::shared_ptr<CANBus> m_Bus;
		std::function<ReceiveCallback> m_Callback;
	};

	class CANBus : public IIOBus, private std::enable_shared_from_this<CANBus>
	{
	public:
		CANBus() = delete;

		CANBus(std::string location, bool isLoopback) :
			m_Location(location), m_isLoopback(isLoopback), m_IsOpened(false)
		{
			Open();
		}
		CANBus(const CANBus& canDevice) = delete;
		~CANBus()
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
		bool IsOpened() const noexcept { return m_IsOpened; }

		bool Open()
		{
			struct ifreq ifr;
			struct sockaddr_can addr; //设置can设备
			int loopback = m_isLoopback ? 1 : 0;
			m_FD = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW); // 非阻塞模式
			strcpy(ifr.ifr_name, m_Location.c_str()); //指定can设备
			if (ioctl(m_FD, SIOCGIFINDEX, &ifr) < 0)
			{
				throw std::runtime_error("ioctl error");
				m_IsOpened = false;
				return false;
			}
			addr.can_family = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;
			if (bind(m_FD, (struct sockaddr*) & addr, sizeof(addr)) < 0) // 绑定Socket
			{
				throw std::runtime_error("bind error");
				m_IsOpened = false;
				return false;
			}
			if (setsockopt(m_FD, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
			{
				throw std::runtime_error("setsockopt loopback error");
				m_IsOpened = false;
				return false;
			}
			UpdateFilter();
			m_IsOpened = true;
			return true;
		}

		bool Close()
		{
			close(m_FD); //关闭套接字
			m_IsOpened = false;
			return true;
		}

		std::shared_ptr<BaseDevice> AddDevice(unsigned int id,
			std::function<ReceiveCallback> callback)
		{
			auto it = m_DeviceMap.find(id);
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

		void Read()
		{
			if (true == m_IsOpened)
			{
				struct can_frame rawFrame;
				while (1)
				{
					int	nbytes = read(m_FD, &rawFrame, sizeof(rawFrame));
					if ((nbytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待接收
					{
						continue;//继续接收数据
					}
					break;//跳出接收循环
				}

				unsigned int id = rawFrame.can_id;
				size_t length = rawFrame.can_dlc;
				auto buffer = std::make_shared<uint8_t[]>(rawFrame.can_dlc);
				memcpy(buffer.get(), rawFrame.data, length);

				auto it = m_DeviceMap.find(id);
				if (it != m_DeviceMap.end())
				{
					it->second->Invoke(length, buffer);
				}
			}
		}

		void WriteRaw(unsigned int can_id, size_t length, std::shared_ptr<uint8_t[]> data)
		{
			if (true == m_IsOpened)
			{
				struct can_frame rawFrame;
				rawFrame.can_id = can_id;
				rawFrame.can_dlc = length;
				memcpy(rawFrame.data, data.get(), length);
				while (1)
				{
					int	nbytes = write(m_FD, &rawFrame, sizeof(rawFrame));
					if ((nbytes < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码都说明还有数据待处理
					{
						continue;//继续接收数据
					}
					break;//跳出接收循环
				}
			}
		}

		std::vector<std::shared_ptr<BaseDevice>> GetDevices()
		{
			std::vector<std::shared_ptr<BaseDevice>> devices;
			for (auto it = m_DeviceMap.begin(); it != m_DeviceMap.end(); ++it)
			{
				devices.push_back(it->second);
			}
			return std::move(devices);
		}
		
	private:
		bool m_IsOpened;
		bool m_isLoopback;
		FileDescriptor m_FD;
		std::string m_Location;
		std::unordered_map<unsigned int, std::shared_ptr<CANDevice>> m_DeviceMap;
		void UpdateFilter()
		{
			size_t size = m_DeviceMap.size();
			std::vector<struct can_filter> rfilters;
			for (auto&& it : m_DeviceMap)
			{
				struct can_filter rf;
				rf.can_id = it.first;
				rf.can_mask = (CAN_EFF_FLAG | CAN_SFF_MASK); // Standard frame
				rfilters.push_back(rf);
			}
			setsockopt(m_FD, SOL_CAN_RAW, CAN_RAW_FILTER, rfilters.data(), sizeof(struct can_filter) * size); // 设置过滤规则
		}
	};

	class CANManager : public IIOManager
	{
	public:

		CANManager()
		{
		}

		IOType Type() const noexcept { return IOType::CAN; }

		// 注册设备
		std::shared_ptr<IIOBus> AddBus(std::string location)
		{
			return AddBus(location, false);
		}
		std::shared_ptr<IIOBus> AddBus(std::string location, bool isLoopback)
		{
			auto device = std::make_shared<CANBus>(location, isLoopback);
			m_BusMap.insert(std::make_pair(location, device));
			return device;
		}

		bool DelBus(std::shared_ptr<IIOBus> bus)
		{
			return m_BusMap.erase(bus->Location());
		}
		bool DelBus(std::string location)
		{
			return m_BusMap.erase(location);
		}

		void WriteTo(std::shared_ptr<BaseDevice> device, size_t length, std::shared_ptr<uint8_t[]> data)
		{
			device->WriteRaw(length, data);
		}

		std::shared_ptr<BaseDevice> AddDevice(std::shared_ptr<CANBus> bus, unsigned int id,
			std::function<ReceiveCallback> callback)
		{
			return bus->AddDevice(id, callback);
		}

		std::shared_ptr<IIOBus> Bus(std::string location)
		{
			auto it = m_BusMap.find(location);
			if (it == m_BusMap.end())
			{
				throw std::runtime_error("No such CAN device");
				return nullptr;
			}
			return it->second;
		}

		std::vector<std::shared_ptr<IIOBus>> GetBuses()
		{
			std::vector<std::shared_ptr<IIOBus>> buses;
			for (auto it = m_BusMap.begin(); it != m_BusMap.end(); ++it)
			{
				buses.push_back(it->second);
			}
			return std::move(buses);
		}
	private:
		std::unordered_map<std::string, std::shared_ptr<IIOBus>> m_BusMap;
	};
} // ossian

#endif // __linux__
#endif // OSSIAN_CORE_IO_CAN
