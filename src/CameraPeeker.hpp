
#ifndef OSSIAN_CAMERA_PEEKER
#define OSSIAN_CAMERA_PEEKER


#include <ossian/Factory.hpp>
#include <ossian/IOListener.hpp>
#include <ossian/Pipeline.hpp>
#include <spdlog/spdlog.h>
#include <HKCamera.hpp>
#include <cuda_runtime.h>
#include <thread>

#include "Aimbot.hpp"

class CameraPeeker : public ossian::IExecutable
{
public:
	OSSIAN_SERVICE_SETUP(CameraPeeker(Aimbot* aimbot))
		: m_Aimbot(aimbot)
	,m_Camera(0,1440,1080)
	{
		int deviceCount = 0;
		cudaError_t cudaStatus; 
		cudaStatus = cudaGetDeviceCount(&deviceCount);
		if (cudaStatus != cudaSuccess)
			throw std::runtime_error("cudaGetDeviceCount() Failed!");
		spdlog::info("CudaEnabledDeviceCount={}", deviceCount);
		cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
		cudaStatus = cudaSetDevice(0);
		if (cudaStatus != cudaSuccess)
			throw std::runtime_error("cudaSetDevice() Failed!");
		m_Camera.SetReceiveImageCallback([this](unsigned char* data, MV_FRAME_OUT_INFO_EX* pFrameInfo)
										 {
											 cv::cuda::GpuMat image;
											 HKCamera::ConvertDataToMat(pFrameInfo, data, image);
											 m_Aimbot->Process(image);
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