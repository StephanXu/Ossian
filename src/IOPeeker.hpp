#ifndef OSSIAN_IO_PEEKER_HPP
#define OSSIAN_IO_PEEKER_HPP

#include <ossian/Factory.hpp>
#include <ossian/IOListener.hpp>
#include <ossian/Pipeline.hpp>
#include <spdlog/spdlog.h>

#include <thread>

class IOPeeker : public ossian::IExecutable
{
public:
	OSSIAN_SERVICE_SETUP(IOPeeker())
		: m_Listener()
	{
	};

	auto ExecuteProc() -> void override
	{
		while (true)
		{
			spdlog::info("hello");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			//m_Listener->Listen(1000);
		}
	}

private:
	ossian::IOListener* m_Listener;
};

#endif // OSSIAN_IO_PEEKER_HPP
