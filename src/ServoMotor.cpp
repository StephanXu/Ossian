#if defined(__linux__) || defined(__APPLE__)
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#endif // __linux__ || __APPLE__
#include <spdlog/spdlog.h>

#include <iostream>
#include <fstream>
#include <csignal>

#include "ServoMotor.hpp"

namespace detail
{
#pragma pack(push, 1)

struct ServoMotorControl
{
	uint8_t m_Channel;
	double m_Value;
};

#pragma pack(pop)
} // detail


ServoMotor::ServoMotor()
{
	// Ignore SIGPIPE to avoid terminating process after reader closed.
	std::signal(SIGPIPE, [](int)
	{
	});
	if (-1 == mkfifo(PIPE_NAME, 0666))
	{
		SPDLOG_CRITICAL("ServoMotor: Create fifo failed");
		return;
	}
	OpenFifo();
}


auto ServoMotor::SetPWMValue(const uint8_t channel, const double value) -> void
{
	if (!IsValid())
	{
		return;
	}
	detail::ServoMotorControl buffer{channel, value};

	if (0 >= write(m_FD, reinterpret_cast<uint8_t*>(&buffer), sizeof(detail::ServoMotorControl)))
	{
		if (errno == EPIPE)
		{
			// Try to reopen file
			SPDLOG_WARN("ServoMotor: Fifo reader terminated, try to reopen file");
			close(m_FD);
			OpenFifo();
		}
	}
}

auto ServoMotor::IsValid() const -> bool
{
	return m_Valid;
}

auto ServoMotor::OpenFifo() -> void
{
	m_Valid = false;
	int fd  = open(PIPE_NAME, O_RDWR);
	if (fd < 0)
	{
		SPDLOG_CRITICAL("ServoMotor: Create fifo FD failed");
		return;
	}
	m_FD    = fd;
	m_Valid = true;
}
