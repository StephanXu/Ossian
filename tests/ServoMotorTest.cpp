#if defined(__linux__) || defined(__APPLE__)
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#endif // __linux__ || __APPLE__

#include <iostream>
#include <fstream>
#include <thread>
#include <csignal>

#pragma pack(push, 1)

struct ServoMotorControl
{
	uint8_t m_Channel;
	double m_Value;
};

#pragma pack(pop)

static void SigpipeHandler(int fd)
{
}

int main()
{
	const char* pipeName = "servo_motor_test_pipe";
	
	ServoMotorControl buffer;
	int a = 0, b = 1;

	mkfifo(pipeName, 0666);
	int fd = open(pipeName, O_RDWR);
	if (fd < 0)
	{
		std::cerr << "Open servo_motor_test_pipe failed" << std::endl;
		return 0;
	}
	while (true)
	{
		buffer.m_Channel = a++;
		buffer.m_Value   = b++;
		auto ret         = write(fd, reinterpret_cast<char*>(&buffer), sizeof(buffer));
		if (0 >= ret)
		{
			if (errno == EPIPE)
			{
				close(fd);
				fd = open(pipeName, O_RDWR);
			}
		}
		std::cout << "Ret: " << ret << " Write {" << (int)buffer.m_Channel << ", " << (float)buffer.m_Value << "}"
			<< std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	close(fd);
}
