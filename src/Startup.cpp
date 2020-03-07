
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
#include "OnlineDebug.hpp"
#include "Remote.hpp"
#include "Capacitor.hpp"

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
		.LoadFromUrl<OssianConfig::Configuration>("ossian.mrxzh.com", 80, "/config");
	app.AddService<OnlineDebug>(
		[](OnlineDebug& option)
		{
			option.Connect("http://ossian.mrxzh.com/logger");
			option.StartLogging("OnlineLog",
								"OssianLog",
								"A piece of log.");
		});
	app.AddService<RoboStatus>();
	app.AddService<VideoInputSource>().AsInputAdapter();
	app.AddService<SerialPortIO>();
	app.AddService<ossian::BaseHardwareManager, ossian::CANManager>();
	app.AddService<ossian::BaseHardwareManager, ossian::UARTManager>();
	app.AddService<ossian::IOListener>();
	app.AddService<IRemote, RemoteSt>(
		[](IRemote& option)
		{
			option.AddRemote("/dev/ttyS0");
		});
	app.AddService<ICapacitor, CapacitorSt>(
		[](ICapacitor& option)
		{
			option.AddCapacitor("can0", 0x211, 0x210);
		});
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
