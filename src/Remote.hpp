#ifndef OSSIAN_REMOTE_HPP
#define OSSIAN_REMOTE_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/MultiThread.hpp>

#include <mutex>

struct RemoteStatus
{
	/* rocker channel information */
	int16_t ch[5];

	/* left and right lever information */
	uint8_t sw[2];
};

class IRemote
{
public:
	virtual ~IRemote() = 0;
	virtual auto AddRemote(std::string location)->void = 0;
	virtual auto Status()->RemoteStatus = 0;
	virtual auto StatusRef()->const RemoteStatus & = 0;
	virtual auto Lock()->void = 0;
	virtual auto UnLock()->void = 0;
	virtual auto TryLock()->bool = 0;
};

template<typename Mutex = std::mutex>
class Remote final : public IRemote
{
	ossian::UARTManager* m_UARTManager;
	RemoteStatus m_Status;
	Mutex m_Mutex;
public:
	OSSIAN_SERVICE_SETUP(Remote(ossian::UARTManager* uartManager))
		:m_UARTManager(uartManager)
	{
	}

	virtual ~Remote() = default;

	auto AddRemote(std::string location)->void override
	{
		m_UARTManager->AddDevice(location)->SetCallback(
			[this](std::shared_ptr<ossian::BaseDevice> device, size_t length, std::shared_ptr<uint8_t[]> data)
			{

				sscanf(reinterpret_cast<const char*>(data.get()),
					   "CH1:%d,CH2:%d,CH3:%d,CH4:%d,CH5:%d,S1:%d,S2:%d",
					   &m_Status.ch[0],
					   &m_Status.ch[1],
					   &m_Status.ch[2],
					   &m_Status.ch[3],
					   &m_Status.ch[4],
					   &m_Status.sw[0],
					   &m_Status.sw[1]);
			});
	}

	auto Status()->RemoteStatus override
	{
		std::lock_guard<Mutex> {m_Mutex};
		return m_Status;
	}

	auto StatusRef()->const RemoteStatus & override
	{
		return m_Status;
	}

	auto Lock()->void override
	{
		m_Mutex.lock();
	}

	auto UnLock()->void override
	{
		m_Mutex.unlock();
	}

	auto TryLock()->bool override
	{
		return m_Mutex.try_lock();
	}
};

using RemoteMt = Remote<std::mutex>;
using RemoteSt = Remote<ossian::null_mutex>;

#endif // OSSIAN_REMOTE_HPP