#include <spdlog/spdlog.h>
#include <HKCamera.hpp>
#include "../src/OnlineDebug.hpp"

#include <thread>
#include <chrono>

int main()
{
	OnlineDebug onlineDbg;
	onlineDbg.Connect("http://ossian.mrxzh.com/logger");
	onlineDbg.StartLogging("OnlineLog",
	                       "Camera",
	                       "Test of HKCamera.");
	spdlog::info("Start testing...");

	HKCamera camera(0, 1440, 1080);
	auto timeStamp = std::chrono::system_clock::now();
	camera.SetReceiveImageCallback(
		[&timeStamp,&camera](unsigned char* data, MV_FRAME_OUT_INFO_EX* pFrameInfo)
		{
			if (pFrameInfo)
			{
				auto currentTime = std::chrono::system_clock::now();
				spdlog::info("GetOneFrame succeed, width:{}, height:{}",
							 pFrameInfo->nWidth,
							 pFrameInfo->nHeight);
				spdlog::info("@Fps=[$FPS={}]",
							 1.0f / static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(currentTime - timeStamp).count()) * 1000000);
				timeStamp = currentTime;
			}
		});
	camera.StartGrabFrame();
	for (;;)
	{
		std::this_thread::yield();
	}
};
