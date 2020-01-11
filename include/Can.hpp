#ifndef NAUTILUS_VISION_DEVICE_IO
#define NAUTILUS_VISION_DEVICE_IO
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

namespace NautilusVision
{
	class IOError :public std::runtime_error
	{
	public:
		IOError(std::string message):std::runtime_error(message){}
	};
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
	class CanBus
	{
	public:
		CanBus() = delete;
		CanBus(std::string location) noexcept :m_Location(location) 
		{
			Open();
		}
		CanBus(const CanBus& canDevice) = delete;
		CanBus(CanBus&& canBus)
		{
			*this = std::move(canBus);
		}
		CanBus& operator=(CanBus&& canBus)
		{
			m_Location = canBus.m_Location;
		}

		~CanBus()
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
			m_Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
			strcpy(ifr.ifr_name, m_Location.c_str()); //指定can设备
			ioctl(m_Socket, SIOCGIFINDEX, &ifr);
			addr.can_family = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;
			bind(m_Socket, (struct sockaddr*) & addr, sizeof(addr)); // 绑定Socket
			setsockopt(m_Socket, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
			UpdateFilter();
			return true;
		}

		void Close()
		{
			close(m_Socket);
		}

		std::string Location() const noexcept { return m_Location; }

		using ReceiveCallback = void(unsigned int id,
									 size_t dataLength,
									 std::shared_ptr<uint8_t[]> data);
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

		FrameData ReadCanRAW()
		{
			// auto frame = std::make_unique<struct CanFrame>();
			
			struct can_frame rawFrame;
			int	nbytes = read(m_Socket, &rawFrame, sizeof(rawFrame));
			unsigned int id = rawFrame.can_id;
			size_t length = rawFrame.can_dlc;
			auto buffer = std::shared_ptr<uint8_t[]>(new uint8_t[rawFrame.can_dlc]);
			memcpy(buffer.get(), rawFrame.data, length);
			return std::move(std::make_tuple(id, length, buffer));
		}
		
		void ReadCan()
		{
			FrameData frameData = ReadCanRAW();//raw frame
			auto it = m_InIdMap.find(std::get<0>(frameData));
			if (m_InIdMap.end() == it)
			{
				return;
			}
			std::apply(it->second, frameData);
		}

		int WriteCanRAW(unsigned int can_id, size_t can_dlc, uint8_t* data)
		{
			struct can_frame rawFrame;
			rawFrame.can_id = can_id;
			rawFrame.can_dlc = can_dlc;
			memcpy(rawFrame.data, data, can_dlc);
			int	nbytes = write(m_Socket, &rawFrame, sizeof(rawFrame));
			if (nbytes < 0)
			{
				throw IOError("Write failed.");
			}
			return nbytes;
		}
		
	protected:
		std::string m_Location;
		std::unordered_map<uint32_t, std::function<ReceiveCallback>> m_InIdMap;
		int m_Socket;
		bool m_isLoopback;

		void UpdateFilter()
		{
			size_t size = m_InIdMap.size();
			std::vector<struct can_filter> rfilters ;
			for (auto&& it : m_InIdMap)
			{
				struct can_filter rf;
				rf.can_id = it.first;
				rf.can_mask = (CAN_EFF_FLAG | CAN_SFF_MASK); // Standard frame
				rfilters.push_back(rf);
			}
			setsockopt(m_Socket, SOL_CAN_RAW, CAN_RAW_FILTER, rfilters.data(), sizeof(struct can_filter)*size); // 设置过滤规则
		}
	};

	class CanManager
	{
	public:
		CanManager()
		{
		}
		void SendTo(std::string location, unsigned int can_id, size_t can_dlc, uint8_t* data)
		{
			auto it = m_DevicesMap.find(location);
			if (m_DevicesMap.end() == it)
			{
				return;
			}
			it->second->WriteCanRAW(can_id, can_dlc, data);
		}
		void AddDevice(std::string location)
		{
			auto device = std::make_unique<CanBus>(location);
			m_DevicesMap.insert(std::make_pair(location, std::move(device)));
		}
		void Destroy(std::string location)
		{
			auto it = m_DevicesMap.find(location);
			if (m_DevicesMap.end() == it)
			{
				return;
			}
			it->second.release();
			m_DevicesMap.erase(it);
		}

		void AddCallback(std::string location, uint32_t id, std::function<CanBus::ReceiveCallback> callback)
		{
			auto it = m_DevicesMap.find(location);
			if (m_DevicesMap.end() == it)
			{
				return;
			}
			it->second->AddReceiveCallback(id, callback);
		}
		std::unique_ptr<CanBus> GetDevice(std::string location)
		{
			auto it = m_DevicesMap.find(location);
			if (m_DevicesMap.end() == it)
			{
				std::abort();
				throw std::runtime_error("No such device");
			}
			return std::move(it->second);
		}
	private:
		std::unordered_map<std::string, std::unique_ptr<CanBus>> m_DevicesMap;
	};

} //NautilusVision

#endif //NAUTILUS_VISION_DEVICE_IO