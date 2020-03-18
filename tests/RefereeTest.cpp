
#include <ossian/io/UART.hpp>
#include <ossian/IOListener.hpp>
#include <spdlog/spdlog.h>

#include <sstream>

#include "../src/OnlineDebug.hpp"

int main()
{
	fmt::print("{:x}\n", 0x12);

	OnlineDebug onlineDbg;
	onlineDbg.Connect("http://ossian.mrxzh.com/logger");
	onlineDbg.StartLogging("OnlineLog",
						   "Referee",
						   "Log of referee test.");
	auto mgr = std::make_shared<ossian::UARTManager>();
	mgr->AddDevice("/dev/ttyS0")->SetCallback(
		[](std::shared_ptr<ossian::UARTDevice> const& device, const size_t length, const uint8_t* data)
		{
			std::stringstream ss;
			for (size_t i{}; i < length; ++i)
			{
				ss << std::hex << data[length];
				ss << " ";
			}
			spdlog::trace("[{}]: {}", length, ss.str());
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