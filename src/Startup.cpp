
#include <mimalloc.h>

#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"
#include "SerialReport.hpp"

#include <thread>

class RoboStatus : public NautilusVision::IOAP::BaseStatus
{
public:
};

void Startup::ConfigServices(AppBuilder &app)
{
	app.InitLog();
	spdlog::info("MI_VERSION:{}", mi_version());

	app.RegisterConfiguration();
    app.RegisterStatusType<RoboStatus>();
    //app.RegisterInputAdapter<CameraInputSource>();
	//app.RegisterInputAdapter<FakeInputSource>();
	app.RegisterInputAdapter<VideoInputSource>();
	app.RegisterService<SerialPortIO>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
	//app.RegisterPipeline<RoboStatus, FakeData,
	//	SerialReport>();
	app.RegisterPipeline<RoboStatus, ImageInputData,
		WindmillDetection>();
	app.Register(WindmillDetection::CreateWindmillDetection);
	//app.Register(Aimbot::CreateAimbot);
	//app.Register(SerialReport::CreateSerialReport);
}
