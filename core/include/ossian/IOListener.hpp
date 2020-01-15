#ifndef NAUTILUS_VISION_DEVICE_IOLISTENER
#define NAUTILUS_VISION_DEVICE_IOLISTENER
#ifdef __linux__

#include "CAN.hpp"
#include "UART.hpp"
#include <sys/epoll.h>
#include <unordered_map>
#include <exception>
#include <string>
namespace NautilusVision
{
	enum class IOType
	{
		CAN,
		UART
	};
	struct CallbackData
	{
		void* p; //对象指针
		IOType type;
	};
	class IOListener
	{
	public:
		IOListener()
		{
			m_EpollFD = epoll_create(MAX_EVENTS); //创建一个Epoll
		}
		const size_t MAX_EVENTS = 128; //应该写在配置文件当中
		void Listen(long timeout)
		{
			struct epoll_event events[MAX_EVENTS + 1];
			int nfd = epoll_wait(m_EpollFD, events, MAX_EVENTS + 1, timeout);
			if (nfd < 0)
			{
				throw std::runtime_error("epoll_wait error"); // 需要Catch
			}
			for (int i = 0; i < nfd; i++)
			{
				//读事件
				if ((events[i].events & EPOLLIN))
				{
					struct CallbackData* pData = static_cast<CallbackData*>(events[i].data.ptr);
					switch (pData->type)
					{
					case IOType::CAN:
					{
						auto dev = reinterpret_cast<CANBus*>(pData->p);
						dev->Read();
						break;
					}
					case IOType::UART:
					{
						auto dev = reinterpret_cast<UART*>(pData->p);
						dev->Read();
						break;
					}
					}
				}
			}
		}
		// 注册监听
		void AddListener(IOType type, std::string location, uint32_t id, std::function<ReceiveCallback> callback)
		{
			int fd;
			std::unordered_map<int, std::unique_ptr<CallbackData>>::iterator it;
			std::unique_ptr<CallbackData> pData;
			switch (type)
			{
			case IOType::CAN:
				fd = m_CANMgr->DeviceFD(location);
				it = m_FDRegistered.find(fd);
				if (it == m_FDRegistered.end())
				{
					pData = std::make_unique<CallbackData>();
					pData->p = m_CANMgr->FindDevice(location);
					pData->type = IOType::CAN;
					AddEpoll(fd, std::move(pData));
				}
				m_CANMgr->AddCallback(location, id, callback);
				break;
			case IOType::UART:
				fd = m_UARTMgr->DeviceFD(location);
				it = m_FDRegistered.find(fd);
				id = 0;
				if (it == m_FDRegistered.end())
				{
					pData = std::make_unique<CallbackData>();
					pData->p = m_UARTMgr->FindDevice(location);
					pData->type = IOType::UART;
					AddEpoll(fd, std::move(pData));
				}
				m_UARTMgr->AddCallback(location, id, callback);
				break;
			}
		}

		void AddEpoll(int fd, std::unique_ptr<CallbackData> pData)
		{
			struct epoll_event epv;
			int op = 0;
			epv.data.ptr = pData.get();
			epv.events = EPOLLIN;// 目前Epoll只用于监听输入
			m_FDRegistered.insert(std::make_pair(fd, std::move(pData)));
			if (epoll_ctl(m_EpollFD, EPOLL_CTL_ADD, fd, &epv) < 0) // 添加一个节点
			{
				throw std::runtime_error("Epoll add failed");
			}
		}

		// 删除epoll监听
		void DelListener(int fd)
		{
			struct epoll_event epv;
			auto it = m_FDRegistered.find(fd);
			if (it == m_FDRegistered.end())
			{
				throw std::runtime_error("No such fd registered");
			}
			else
			{
				epv.data.ptr = NULL;
				epoll_ctl(m_EpollFD, EPOLL_CTL_DEL, fd, &epv);
				m_FDRegistered.erase(it);
			}
		}

		void SetCANManager(CANManager* mgr) { m_CANMgr = mgr; }
		void SetUARTManager(UARTManager* mgr) { m_UARTMgr = mgr; }
	private:
		int m_EpollFD;
		std::unordered_map<int, std::unique_ptr<CallbackData>> m_FDRegistered;
		CANManager* m_CANMgr;
		UARTManager* m_UARTMgr;
	};
}
#endif // __linux__
#endif // NAUTILUS_VISION_DEVICE_IOLISTENER
