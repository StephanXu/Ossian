#ifndef OSSIAN_IO_PEEKER_HPP
#define OSSIAN_IO_PEEKER_HPP

#include <ossian/Factory.hpp>
#include <ossian/IOListener.hpp>
#include <ossian/Pipeline.hpp>
#include <spdlog/spdlog.h>

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
		std::this_thread::sleep_for(std::chrono::seconds(5));
		spdlog::info("IMU Calib Done.");
		while (true)
		{
			m_Listener->Listen(EpollIndex, 1000);
		}
	}

private:
	ossian::IOListener* m_Listener;
};

#endif // OSSIAN_IO_PEEKER_HPP
