
#ifndef OSSIAN_CAMERA_PEEKER
#define OSSIAN_CAMERA_PEEKER


#include <ossian/Factory.hpp>
#include <ossian/IOListener.hpp>
#include <ossian/Pipeline.hpp>
#include <spdlog/spdlog.h>
#include <HKCamera.hpp>
#include <thread>

#include "Aimbot.hpp"

class CameraPeeker : public ossian::IExecutable
{
public:
	OSSIAN_SERVICE_SETUP(CameraPeeker(Aimbot* aimbot))
		: m_Aimbot(aimbot)
	,m_Camera(0,1440,1080)
	{
		auto timeStamp = std::chrono::system_clock::now();
		m_Camera.SetReceiveImageCallback([this, &timeStamp](unsigned char* data, MV_FRAME_OUT_INFO_EX* pFrameInfo)
										 {
											 //cv::cuda::GpuMat image;
											 //HKCamera::ConvertDataToMat(pFrameInfo, data, image);
											 if (pFrameInfo)
											 {
												auto currentTime = std::chrono::system_clock::now();
												std::cerr << "Read Fps = " <<
													1.0f / static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(currentTime - timeStamp).count()) * 1000000
														<< std::endl;
												timeStamp = currentTime;
												m_Aimbot->Process(data);
											 }
											 
										 });
		
	};

	auto ExecuteProc() -> void override
	{
		m_Camera.StartGrabFrame();
		while (true)
		{
			std::this_thread::yield();
		}
	}

private:
	Aimbot* m_Aimbot;
	HKCamera m_Camera;
};

#endif // OSSIAN_CAMERA_PEEKER