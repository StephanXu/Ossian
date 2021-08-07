#include <spdlog/spdlog.h>
#include "../src/OnlineDebug.hpp"

#include <random>
#include <string>

int main()
{
	const std::string demoArgumentId{"607707c58b3d8600013b8c96"};
	OnlineDebug onlineDbg;
	onlineDbg.Connect("http://ossian.mrxzh.com/logger");
	onlineDbg.StartLogging("OnlineLog",
	                       "OnlineDebuggerTest",
	                       "Test of online debugger",
	                       demoArgumentId,
	                       true,
	                       0,
	                       true,
	                       "offline-log.log");

	std::default_random_engine random(std::time(nullptr));
	std::uniform_real_distribution<double> uniformDist(-10, 10);
	while (true)
	{
		for (int i{}; i < 100; ++i)
		{
			spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
			            "@Fig1=[$var1={}]",
			            1);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
			            "@Fig1=[$var2={}]",
			            2);
		}
		for (int i{}; i < 3; ++i)
		{
			spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
			            "@Fig1=[$var1={}]",
			            0.5);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
			            "@Fig1=[$var2={}]",
			            1);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}
