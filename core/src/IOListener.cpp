#include "ossian/IOListener.hpp"

#ifdef __linux__
#include <sys/epoll.h>

namespace ossian
{
IOListener::IOListener(std::vector<IListenable*>* ioBuses)
{
	m_EpollFD = epoll_create(MAX_EVENTS); //����һ��Epoll
	for (auto item : *ioBuses) { AddBus(item); }
}

void IOListener::Listen(const long timeout) const
{
	struct epoll_event events[MAX_EVENTS + 1];
	const auto nfd = epoll_wait(m_EpollFD, events, MAX_EVENTS, timeout);
	if (nfd < 0)
	{
		throw std::runtime_error("epoll_wait error"); // ��ҪCatch
	}
	for (auto i = 0; i < nfd; i++)
	{
		//���¼�
		if ((events[i].events & EPOLLIN))
		{
			auto pData = static_cast<CallbackData*>(events[i].data.ptr);
			pData->bus->Read();
		}
	}
}

void IOListener::AddEpoll(const FileDescriptor fd, std::unique_ptr<CallbackData> pData)
{
	struct epoll_event epv;
	epv.data.ptr = pData.get();
	epv.events = EPOLLIN; // Ŀǰepollֻ���ڼ�������
	m_FDRegistered.insert(std::make_pair(fd, std::move(pData)));
	if (epoll_ctl(m_EpollFD, EPOLL_CTL_ADD, fd, &epv) < 0) // ���һ���ڵ�
	{
		throw std::runtime_error("Epoll add failed");
	}
}

void IOListener::DelEpoll(const FileDescriptor fd)
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

bool IOListener::AddBus(IListenable* const bus)
{
	auto it = m_Buses.find(bus);
	if (it != m_Buses.end())
	{
		throw std::runtime_error("Bus already exist");
		return false;
	}
	m_Buses.insert(bus);
	// ��FDע�ᵽEpoll��
	auto fd = bus->FD();
	auto pData = std::make_unique<CallbackData>();
	pData->bus = bus;
	AddEpoll(fd, std::move(pData));
	return true;
}

bool IOListener::DelBus(IListenable* const bus)
{
	auto it = m_Buses.find(bus);
	if (it == m_Buses.end())
		return false; // ������
	DelEpoll(bus->FD());
	m_Buses.erase(it);
	return true;
}
} // ossian
#endif // __linux__