
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
#include "Referee.hpp"

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

	app.AddService<ossian::CANManager>();
	app.AddService<ossian::UARTManager>();
	app.AddService<ossian::IOListener>();
	app.AddService<ossian::MotorManager>();

	app.AddService<IReferee, Referee>(
		[](IReferee& option)
		{
			option.AddReferee("/dev/ttyS1");
		});
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

	app.AddService<Chassis>(
		[](Chassis& option)
		{
			option.AddMotor(Chassis::MotorPosition::LF, "can0", 1, 0x1ff);
			option.AddMotor(Chassis::MotorPosition::LR, "can0", 2, 0x1ff);
			option.AddMotor(Chassis::MotorPosition::RF, "can0", 3, 0x1ff);
			option.AddMotor(Chassis::MotorPosition::RR, "can0", 4, 0x1ff);
		});
	app.AddService<Gimbal>(
		[](Gimbal& option)
		{
			option.AddMotor(Gimbal::MotorPosition::Pitch, "can1", 1, 0x1ff);
			option.AddMotor(Gimbal::MotorPosition::Yaw, "can1", 1, 0x1ff);
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
