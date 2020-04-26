#include <ossian/io/CAN.hpp>
#include <ossian/IOListener.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <sstream>

#include "../src/OnlineDebug.hpp"
#include "ossian/io/UART.hpp"
/*
 * 宏定义说明
 * METHOD_EPOLL 采用epoll机制测试
 * METHOD_RAW	采用原生读取方式测试
 * 请不要同时定义
 *
 * 参数说明
 * location		总线地址
 * deviceId		设备ID
 * epollTimeout epoll超时时间
 */
#define METHOD_RAW
std::chrono::time_point<std::chrono::system_clock> now, prev;

int main()
{
	const std::string location  = "can1";
	const unsigned int deviceId = 0x20a;
	const int epollTimeout      = 1;
	prev                        = std::chrono::system_clock::now();
	OnlineDebug onlineDbg;
	onlineDbg.Connect("http://ossian.mrxzh.com/logger");
	onlineDbg.StartLogging("OnlineLog",
	                       "CAN",
	                       "Log of IO Interval test.");
	SPDLOG_TRACE("Start testing...");
	ossian::IOListener listener;
	auto mgr = std::make_shared<ossian::CANManager>(&listener);
	mgr->AddDevice(location, deviceId)
	   ->SetCallback(
		   [](std::shared_ptr<ossian::CANDevice> const& device, const size_t length, const uint8_t* data)
		   {
			   prev          = now;
			   now           = std::chrono::system_clock::now();
			   auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - prev).count();
			   SPDLOG_TRACE("Time elapsed {} microseconds.", duration);
			   SPDLOG_INFO("@CANTime=[$time={}]", duration);
		   });
	auto bus = mgr->Bus(location);
	while (true)
	{
#ifdef METHOD_EPOLL
		listener.Listen(1, epollTimeout);
#endif //METHOD_EPOLL
#ifdef METHOD_RAW
		bus->Read();
#endif // METHOD_RAW
	}
}
