#include "CameraPeeker.hpp"

#include <thread>

CameraPeeker::CameraPeeker(IVisionModeSwitcher* switcher)
	: m_Camera(0, 1440, 1080)
{
	m_Camera.SetReceiveImageCallback(
		[switcher](unsigned char* data, MV_FRAME_OUT_INFO_EX* pFrameInfo)-> void
		{
			switcher->Process(data);
		});
}

auto CameraPeeker::ExecuteProc() -> void
{
	m_Camera.StartGrabFrame();
	while (true)
	{
		std::this_thread::yield();
	}
};
