#include "ossian/IOListener.hpp"

#ifdef __linux__
#include <sys/epoll.h>

namespace ossian
{
IOListener::IOListener()
{
	for (size_t i = 0; i < EPOLL_SIZE; ++i)
	{
		m_EpollFD[i] = epoll_create(MAX_EVENTS); //创建一个Epoll
	}
}

void IOListener::Listen(const size_t epollIndex, const long timeout) const
{
	struct epoll_event events[MAX_EVENTS + 1];
	const auto nfd = epoll_wait(m_EpollFD[epollIndex], events, MAX_EVENTS, timeout);
	if (nfd < 0)
	{
		throw std::runtime_error("epoll_wait error"); // 需要Catch
	}
	for (auto i = 0; i < nfd; i++)
	{
		//读事件
		if ((events[i].events & EPOLLIN))
		{
			auto pData = static_cast<CallbackData*>(events[i].data.ptr);
			pData->bus->Read();
		}
	}
}

void IOListener::AddEpoll(const size_t epollIndex, const FileDescriptor fd, std::unique_ptr<CallbackData> pData)
{
	struct epoll_event epv;
	epv.data.ptr = pData.get();
	epv.events   = EPOLLIN; // 目前epoll只用于监听输入
	m_FDRegistered[epollIndex].insert(std::make_pair(fd, std::move(pData)));
	if (epoll_ctl(m_EpollFD[epollIndex], EPOLL_CTL_ADD, fd, &epv) < 0) // 添加一个节点
	{
		throw std::runtime_error("Epoll add failed");
	}
}

void IOListener::DelEpoll(const size_t epollIndex, const FileDescriptor fd)
{
	struct epoll_event epv;
	auto it = m_FDRegistered[epollIndex].find(fd);
	if (it == m_FDRegistered[epollIndex].end())
	{
		throw std::runtime_error("No such fd registered");
	}
	else
	{
		epv.data.ptr = NULL;
		epoll_ctl(m_EpollFD[epollIndex], EPOLL_CTL_DEL, fd, &epv);
		m_FDRegistered[epollIndex].erase(it);
	}
}

bool IOListener::AddBus(const size_t epollIndex, IListenable* const bus)
{
	auto it = m_Buses[epollIndex].find(bus);
	if (it != m_Buses[epollIndex].end())
	{
		throw std::runtime_error("Bus already exist");
		return false;
	}
	m_Buses[epollIndex].insert(bus);
	// 将FD注册到Epoll中
	auto fd    = bus->FD();
	auto pData = std::make_unique<CallbackData>();
	pData->bus = bus;
	AddEpoll(epollIndex, fd, std::move(pData));
	return true;
}

bool IOListener::DelBus(const size_t epollIndex, IListenable* const bus)
{
	auto it = m_Buses[epollIndex].find(bus);
	if (it == m_Buses[epollIndex].end())
		return false; // 不存在
	DelEpoll(epollIndex, bus->FD());
	m_Buses[epollIndex].erase(it);
	return true;
}
} // ossian
#endif // __linux__
