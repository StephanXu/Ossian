#include "ossian/io/CAN.hpp"
#include "ossian/io/UART.hpp"
#include "ossian/IOListener.hpp"
#include "ossian/io/IO.hpp"
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include <vector>
using namespace ossian;
using namespace std;

void func(shared_ptr<CANDevice> const& device, const size_t length, const uint8_t* data)
{
	SPDLOG_INFO("Callback function called with length {}", length);
}

int main()
{
	IOListener listener;
	auto mgr = std::make_shared<CANManager>(&listener);
	mgr->AddDevice("can0", 0x100)->SetCallback(func);
	
	while (true)
	{
		listener.Listen(0, 1000);
		listener.Listen(1, 1000);
	}
	return 0;
}