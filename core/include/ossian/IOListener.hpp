#ifndef OSSIAN_CORE_IOLISTENER
#define OSSIAN_CORE_IOLISTENER
#ifdef __linux__

#include "io/IO.hpp"
#include <sys/epoll.h>
#include <unordered_map>
#include <exception>
#include <string>

#include <iostream>
namespace ossian
{
	struct CallbackData
	{
		std::shared_ptr<IIO> io; //对象指针
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
					auto dev = pData->io;
					dev->Read();
				}
			}
		}

		void AddEpoll(int fd, std::unique_ptr<CallbackData> pData)
		{
			struct epoll_event epv;
			epv.data.ptr = pData.get();
			epv.events = EPOLLIN;// 目前epoll只用于监听输入
			m_FDRegistered.insert(std::make_pair(fd, std::move(pData)));
			if (epoll_ctl(m_EpollFD, EPOLL_CTL_ADD, fd, &epv) < 0) // 添加一个节点
			{
				throw std::runtime_error("Epoll add failed");
			}
		}

		// 删除epoll监听
		void DelEpoll(int fd)
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
		// 让一个Manager被epoll接管
		bool AddManager(IIOManager* mgr)
		{
			auto type = mgr->Type();
			auto it = m_IOManagers.find(type);
			if(it != m_IOManagers.end())
			{
				throw std::runtime_error("Manager already exist");
				return false;
			}
			auto fds = mgr->FDs();
			m_IOManagers.insert(std::make_pair(type, mgr));
			// 将所有的IO都注册到Epoll中
			for(auto&& fd : fds)
			{
				auto pData = std::make_unique<CallbackData>();
				pData->io = mgr->FindDevice(fd);
				AddEpoll(fd, std::move(pData));
			}
			return true;
		}
	private:
		int m_EpollFD;
		std::unordered_map<int, std::unique_ptr<CallbackData>> m_FDRegistered;
		std::unordered_map<IOType, IIOManager*> m_IOManagers;
	};
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IOLISTENER
