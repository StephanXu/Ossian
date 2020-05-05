#include "ossian/Configuration.hpp"
#include "../src/OnlineDebug.hpp"
#include "Config.pb.h"

#include <iostream>
#include <string>
#include <thread>
#include <random>

int main()
{
	ossian::Utils::ConfigLoader config;
	OnlineDebug debugger;

	const auto console = spdlog::stderr_color_mt("console");
	spdlog::set_default_logger(console);
	spdlog::set_pattern("[%Y-%m-%dT%T.%e%z] [%-5t] %^[%l]%$ %v");
	spdlog::set_level(spdlog::level::trace);

	config.LoadConfigFromUrl<OssianConfig::Configuration>("ossian.mrxzh.com",
	                                                      80,
	                                                      "/api/argument");

	debugger.Connect("debug.fenzhengrou.wang:5001");
	debugger.StartLoggingAndArchiveLog("OnlineLog",
	                                   "Online Debug Test",
	                                   "Online debug test",
	                                   *config.Instance<OssianConfig::Configuration>());
	std::default_random_engine random(std::time(nullptr));
	std::uniform_real_distribution<double> uniformDist(-10, 10);

	using Clock = std::chrono::high_resolution_clock;
	using TimeStamp = Clock::time_point;
	TimeStamp lastTime = Clock::now();
	while (true)
	{
		while (12000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
		{
			std::this_thread::yield();
		}
		SPDLOG_INFO("@Interval=[$t={}]",
		            std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count());
		lastTime = Clock::now();

		for (size_t i{}; i < 10; ++i)
		{
			spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
			            "@Fig{}=[$var{}1={},$var{}1={}]", i, i,
			            uniformDist(random) / 100.0f, i,
			            uniformDist(random) / 100.0f);
		}
	}
}
