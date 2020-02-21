
#include <ossian/ossian.hpp>
#include <mimalloc.h>
#include "Config.pb.h"

#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"
#include "SerialReport.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"

#include <thread>

class RoboStatus : public ossian::IOAP::BaseStatus
{
public:
	OSSIAN_SERVICE_SETUP(RoboStatus()) {}
};

void Startup::ConfigServices(AppBuilder& app)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	app.InitLog();
	spdlog::info("MI_VERSION:{}", mi_version());

	app.AddService<Utils::ConfigLoader>()
		.LoadFromUrl<OssianConfig::Configuration>("mrxzh.com", 5000, "/config");
	app.AddService<RoboStatus>();
	app.AddService<VideoInputSource>().AsInputAdapter();
	app.AddService<SerialPortIO>();
	app.AddService<ossian::IIOManager, ossian::CANManager>(
		[](ossian::CANManager& mgr)
		{
			mgr.AddBus("can0", true);
		});
	app.AddService<ossian::IOListener>();
	//app.AddInputAdapter<CameraInputSource>();
	//app.AddInputAdapter<FakeInputSource>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
	//app.RegisterPipeline<RoboStatus, FakeData,
	//	SerialReport>();

	app.RegisterPipeline<RoboStatus, ImageInputData,
		WindmillDetection>();
	app.Add<WindmillDetection>(WindmillDetection::CreateWindmillDetection);

	//app.Register(Aimbot::CreateAimbot);
	//app.Register(SerialReport::CreateSerialReport);
}
