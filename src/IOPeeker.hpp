#ifndef OSSIAN_IO_PEEKER_HPP
#define OSSIAN_IO_PEEKER_HPP

#include <ossian/Factory.hpp>
#include <ossian/IOListener.hpp>
#include <ossian/Pipeline.hpp>
#include <spdlog/spdlog.h>

#include "ossian/io/UART.hpp"
#include <thread>

template <size_t EpollIndex>
class IOPeeker : public ossian::IExecutable
{
public:
	OSSIAN_SERVICE_SETUP(IOPeeker(ossian::IOListener* listener))
		: m_Listener(listener)
	{
	};

	auto ExecuteProc() -> void override
	{
		while (true)
		{
			m_Listener->Listen(EpollIndex, 1000);
		}
	}

private:
	ossian::IOListener* m_Listener;
};

class IOWorker : public ossian::IExecutable
{
public:
	OSSIAN_SERVICE_SETUP(IOWorker(ossian::UARTManager* uartmgr))
		: m_UARTManager(uartmgr)
	{};

	auto ExecuteProc() -> void override
	{
		while (true)
		{
			auto devices = m_UARTManager->GetDevices();
			for (auto dev : devices)
			{
				dev->Process();
			}
		}
	}

private:
	ossian::UARTManager* m_UARTManager;
};

#endif // OSSIAN_IO_PEEKER_HPP
