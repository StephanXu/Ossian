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

	debugger.Connect("rpc.mrxzh.com");
	debugger.StartLoggingAndArchiveLog("OnlineLog",
	                                   "Online Debug Test",
	                                   "Online debug test",
	                                   *config.Instance<OssianConfig::Configuration>());
	std::default_random_engine random(std::time(nullptr));
	std::uniform_real_distribution<double> uniformDist(-10, 10);
	while (true)
	{
		spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
		            "@Fig1=[$var1={},$var2={}]",
		            uniformDist(random) / 100.0f,
		            uniformDist(random) / 100.0f);
		spdlog::log(static_cast<spdlog::level::level_enum>(random() % 6),
		            "@Fig2=[$var1={},$var2={}]",
		            uniformDist(random) / 100.0f,
		            uniformDist(random) / 100.0f);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}
