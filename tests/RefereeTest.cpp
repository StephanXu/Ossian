﻿#include <ossian/io/UART.hpp>
#include <ossian/IOListener.hpp>
#include <spdlog/spdlog.h>

#include <sstream>

#include "../src/OnlineDebug.hpp"

int main()
{
	OnlineDebug onlineDbg;
	onlineDbg.Connect("http://ossian.mrxzh.com/logger");
	onlineDbg.StartLogging("OnlineLog",
	                       "Referee",
	                       "Log of referee test.");
	spdlog::info("Start testing...");
	auto mgr = std::make_shared<ossian::UARTManager>();
	mgr->AddDevice("/dev/ttyTHS2")->SetCallback(
		[](std::shared_ptr<ossian::UARTDevice> const& device, const size_t length, const uint8_t* data)
		{
			std::stringstream ss;
			for (size_t i{}; i < length; ++i)
			{
				ss << fmt::format("{:02x}", data[i]);
				ss << " ";
			}
			spdlog::info("[{}]: {}", length, ss.str());
		});
	auto buses = mgr->GetBuses();
	std::vector<ossian::IListenable*> lis;
	for (auto bus : buses)
	{
		lis.push_back(bus);
	}
	ossian::IOListener listener(&lis);
	while (true)
	{
		listener.Listen(1000);
	}
}
