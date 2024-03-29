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
	SPDLOG_TRACE("Start testing...");
	ossian::IOListener listener;
	auto mgr = std::make_shared<ossian::UARTManager>(&listener);
	using namespace ossian::UARTProperties;
	mgr->AddDevice("/dev/ttyTHS2",
	               115200,
	               FlowControl::FlowControlNone,
	               DataBits::DataBits8,
	               StopBits::StopBits1,
	               Parity::ParityNone)
	   ->SetCallback(
		   [](std::shared_ptr<ossian::UARTDevice> const& device, const size_t length, const uint8_t* data)
		   {
#if (SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE)
			   std::stringstream ss;
			   for (size_t i{}; i < length; ++i)
			   {
				   ss << fmt::format("{:02x}", data[i]);
				   ss << " ";
			   }
			   SPDLOG_INFO("[{}]: {}", length, ss.str());
#endif
		   });	
	while (true)
	{
		listener.Listen(0, 1000);
	}
}
