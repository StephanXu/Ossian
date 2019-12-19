
#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"

class RoboStatus : public NautilusVision::IOAP::BaseStatus
{
public:
};

void Startup::ConfigServices(AppBuilder &app)
{
	app.InitLog();
	app.RegisterConfiguration();
    app.RegisterStatusType<RoboStatus>();
    app.RegisterInputAdapter<CameraInputSource>();
	app.RegisterService<SerialPortIO>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
	app.RegisterPipeline<RoboStatus, ImageInputData,
		Aimbot>();
	//app.Register(WindmillDetection::CreateWindmillDetection);
	app.Register(Aimbot::CreateAimbot);
}
