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
#include "IODeviceMap.hpp"
namespace ossian
{

	/*
	 * Controller Area Network Identifier structure
	 *
	 * bit 0-28	: CAN identifier (11/29 bit)
	 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
	 * bit 30	: remote transmission request flag (1 = rtr frame)
	 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
	 */
	using FrameData = std::tuple<unsigned int, size_t, std::shared_ptr<uint8_t[]>>;
	// 0->CAN ID 1->数据长度 2->数据
	class CANBus : public IIO
	{
	public:
		CANBus() = delete;
		CANBus(std::string location, bool isLoopback) noexcept :m_isLoopback(isLoopback)
		{
			m_Location = location;
			Open();
		}
		CANBus(const CANBus& canDevice) = delete;
		CANBus(CANBus&& canBus)
		{
			*this = std::move(canBus);
		}
		CANBus& operator=(CANBus&& canBus)
		{
			m_Location = canBus.m_Location;
			return *this;
		}
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
				return false;
			}
			addr.can_family = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;
			if (bind(m_FD, (struct sockaddr*) & addr, sizeof(addr)) < 0) // 绑定Socket
			{
				throw std::runtime_error("bind error");
				return false;
			}
			if (setsockopt(m_FD, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
			{
				throw std::runtime_error("setsockopt loopback error");
				return false;
			}
			UpdateFilter();
			return true;
		}

		bool Close()
		{
			close(m_FD); //关闭套接字
			return true;
		}

		bool AddReceiveCallback(uint32_t id, std::function<ReceiveCallback> callback)
		{
			auto it = m_InIdMap.find(id);
			if (m_InIdMap.end() != it)
			{
				return false; //Maybe need exception
			}
			m_InIdMap.insert(std::make_pair(id, callback));
			UpdateFilter();
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
			auto buffer = std::shared_ptr<uint8_t[]>(new uint8_t[rawFrame.can_dlc]);
			memcpy(buffer.get(), rawFrame.data, length);
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

		void WriteRaw(unsigned int can_id, size_t can_dlc, uint8_t* data)
		{
			struct can_frame rawFrame;
			rawFrame.can_id = can_id;
			rawFrame.can_dlc = can_dlc;
			memcpy(rawFrame.data, data, can_dlc);
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

	private:
		bool m_isLoopback;

		void UpdateFilter()
		{
			size_t size = m_InIdMap.size();
			std::vector<struct can_filter> rfilters;
			for (auto&& it : m_InIdMap)
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
		/* 发送原始数据到指定位置 */
		void SendToRaw(std::string location, unsigned int can_id, size_t can_dlc, uint8_t* data)
		{
			auto dev = FindDevice(location);
			dev->WriteRaw(can_id, can_dlc, data);
		}
		/* 从指定位置读取FrameData */
		FrameData ReadFrom(std::string location)
		{
			auto dev = FindDevice(location);
			return dev->Read();
		}
		// 注册设备
		bool AddDevice(std::string location)
		{
			return AddDevice(location, false);
		}
		bool AddDevice(std::string location, bool isLoopback)
		{
			auto device = std::make_shared<CANBus>(location, isLoopback);
			return m_DeviceMap.Insert(device);
		}
		bool DelDevice(std::string location)
		{
			return m_DeviceMap.Erase(location);
		}
		FileDescriptor DeviceFD(std::string location)
		{
			auto dev = FindDevice(location);
			return dev->FD();
		}
		void AddCallback(std::string location, uint32_t id, std::function<ReceiveCallback> callback)
		{
			auto dev = FindDevice(location);
			dev->AddReceiveCallback(id, callback);
		}
		CANBus* FindDevice(std::string location)
		{
			auto it = m_DeviceMap.Find(location);
			if (nullptr == it)
			{
				throw std::runtime_error("No such CAN device");
			}
			return it.get();
		}
		CANBus* FindDevice(FileDescriptor fd)
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
			for(auto it = m_DeviceMap.begin(); it != m_DeviceMap.end(); ++it)
			{
				fd.push_back(it->second->FD());
			}
			return std::move(fd);
		}
	private:
		DeviceMap<CANBus> m_DeviceMap;
	};
} // ossian

#endif // __linux__
#endif // OSSIAN_CORE_IO_CAN