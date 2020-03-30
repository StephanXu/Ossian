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
	OSSIAN_SERVICE_SETUP(IOPeeker(ossian::IOListener* listener))
		: m_Listener(listener)
	{
	};

	auto ExecuteProc() -> void override;

private:
	ossian::IOListener* m_Listener;
};

#endif // OSSIAN_IO_PEEKER_HPP
