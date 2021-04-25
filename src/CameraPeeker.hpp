#ifndef OSSIAN_CAMERA_PEEKER
#define OSSIAN_CAMERA_PEEKER

#include <ossian/Factory.hpp>
#include <ossian/Pipeline.hpp>
#include <HKCamera.hpp>

#include "VisionModeSwitcher.hpp"


class CameraPeeker : public ossian::IExecutable
{
public:
	OSSIAN_SERVICE_SETUP(CameraPeeker(IVisionModeSwitcher* switcher));

	auto ExecuteProc() -> void override;

private:

	HKCamera m_Camera;
};

#endif // OSSIAN_CAMERA_PEEKER
