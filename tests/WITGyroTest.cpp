#include "ossian/io/UART.hpp"
#include "ossian/IOListener.hpp"
#include "ossian/io/IO.hpp"
#include <memory>
#include <spdlog/spdlog.h>
#include <sstream>
#include <vector>
using namespace ossian;
using namespace std;

void func(shared_ptr<UARTDevice> const& device, const size_t length, const uint8_t* data)
{
	std::stringstream ss;
	for (size_t i{}; i < length; ++i)
	{
		ss << fmt::format("{:02x}", data[i]);
		ss << " ";
	}
	spdlog::info("UART: location={} len={} data={}", device->Bus()->Location(),length, ss.str());
}

int main()
{
	IOListener listener;
	auto mgr = std::make_shared<UARTManager>(&listener);
	auto dev = mgr->AddDevice("/dev/ttyUSB1",
				   UARTProperties::R230400,
				   UARTProperties::FlowControlNone,
				   UARTProperties::DataBits8,
				   UARTProperties::StopBits1,
				   UARTProperties::ParityNone)->SetCallback(func);
	uint8_t command[] = { 'A','T','+','E','T','\n'};
	dev->WriteRaw(6, command);
	
	while (true)
	{
		listener.Listen(0, 1000);
	}
	return 0;
}

/*
数据样例
[2020-04-17 10:44:06.180] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e4 ff e2 04 3d 06 55 52 00 00 00 00 00 00 55 53 05 1b 80 00 01 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:07.179] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 ea ff e0 04 3e 06 55 52 00 00 00 00 00 00 55 53 05 1b 80 00 02 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:08.179] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e4 ff df 04 40 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 01 40 55 54 f2 09 00 00 00 00
[2020-04-17 10:44:09.178] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff df 04 3d 06 55 52 00 00 00 00 00 00 55 53 04 1b 81 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:10.177] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff df 04 3d 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 01 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:11.177] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff e0 04 3d 06 55 52 00 00 00 00 00 00 55 53 05 1b 82 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:12.176] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff df 04 3b 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:13.175] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e4 ff df 04 40 06 55 52 00 00 00 00 00 00 55 53 05 1b 82 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:14.174] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff df 04 40 06 55 52 00 00 00 00 00 00 55 53 05 1b 82 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:15.174] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e5 ff e2 04 40 06 55 52 00 00 00 00 00 00 55 53 06 1b 82 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:16.173] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e2 04 3d 06 55 52 00 00 00 00 01 00 55 53 06 1b 81 00 02 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:17.172] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e3 04 3e 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 01 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:18.171] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff e0 04 3d 06 55 52 00 00 00 00 00 00 55 53 06 1b 80 00 01 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:19.171] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e5 ff e0 04 3d 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 01 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:20.170] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e2 04 3d 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:21.169] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e5 ff df 04 3d 06 55 52 00 00 00 00 00 00 55 53 06 1b 80 00 02 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:22.168] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff df 04 3d 06 55 52 00 00 00 00 01 00 55 53 05 1b 80 00 01 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:23.168] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e4 ff dd 04 3e 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 01 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:24.167] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e0 04 3d 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 00 40 55 54 f1 09 00 00 00 00
[2020-04-17 10:44:25.166] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff df 04 38 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 01 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:26.166] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e0 04 3b 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:27.165] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff e0 04 3d 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:28.164] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e2 04 40 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:29.163] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e7 ff df 04 3e 06 55 52 00 00 00 00 00 00 55 53 05 1b 81 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:30.163] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e5 ff df 04 3b 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 01 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:31.162] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e4 ff e2 04 3e 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:32.161] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e8 ff e0 04 3b 06 55 52 00 00 00 00 00 00 55 53 06 1b 81 00 00 40 55 54 f0 09 00 00 00 00
[2020-04-17 10:44:33.160] [info] UART: location=/dev/ttyUSB1 len=32 data=55 51 e4 ff df 04 3d 06 55 52 00 00 00 00 00 00 55 53 06 1b 82 00 00 40 55 54 f0 09 00 00 00 00
 */