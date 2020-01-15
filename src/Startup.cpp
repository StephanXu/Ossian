
#include <ossian/ossian.hpp>
#include <mimalloc.h>
#include "Config.pb.h"

#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"
#include "SerialReport.hpp"

#include <thread>

class RoboStatus : public ossian::IOAP::BaseStatus
{
public:
};

void Startup::ConfigServices(AppBuilder &app)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	app.InitLog();

	spdlog::info("MI_VERSION:{}", mi_version());
	
	app.RegisterConfigLoader<OssianConfig::Configuration>();
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
