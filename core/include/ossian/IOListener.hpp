#ifndef OSSIAN_CORE_IOLISTENER
#define OSSIAN_CORE_IOLISTENER

#ifdef __linux__
#include <unordered_map>
#include <string>
#include <vector>
#include <set>

#include "io/IO.hpp"
#include "Factory.hpp"

namespace ossian
{
struct CallbackData
{
	IListenable* bus; //对象指针
};

class IOListener
{
public:
	static const int MAX_EVENTS = 128;

	IOListener(const IOListener& other) = delete;
	IOListener(IOListener&& other) noexcept = delete;
	IOListener& operator=(const IOListener& other) = delete;
	IOListener& operator=(IOListener&& other) noexcept = delete;
	OSSIAN_SERVICE_SETUP(IOListener());

	void Listen(const long timeout) const;
	// 让一个Bus被epoll接管
	bool AddBus(IListenable* const bus);
	// 从接管的列表中删除Bus
	bool DelBus(IListenable* const bus);

private:
	int m_EpollFD;
	void AddEpoll(const FileDescriptor fd, std::unique_ptr<CallbackData> pData);
	void DelEpoll(const FileDescriptor fd);
	std::unordered_map<int, std::unique_ptr<CallbackData>> m_FDRegistered;
	std::set<IListenable*> m_Buses;
};

class Attachable
{
public:
	Attachable(): m_IsListenerAttached(false){}
	virtual auto AttachListener(IOListener* listener) -> bool
	{
		if(m_IsListenerAttached == true)
		{
			return false; // Listener 已存在
		}
		m_Listener = listener;
		return m_IsListenerAttached = true;
	}
	bool m_IsListenerAttached;
	IOListener* m_Listener;
};

} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IOLISTENER
