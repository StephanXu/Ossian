#include <ossian/ossian.hpp>
#include <mimalloc.h>
#include "Config.pb.h"

#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "OnlineDebug.hpp"
#include "Remote.hpp"
#include "Capacitor.hpp"
#include "Referee.hpp"
#include "Gyro.hpp"
#include "IOPeeker.hpp"
#include "CameraPeeker.hpp"

#include <thread>

void Startup::ConfigServices(AppBuilder& app)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	app.InitLog();
	spdlog::info("MI_VERSION: {}", mi_version());

	app.AddService<Utils::ConfigLoader>()
	   .LoadFromUrl<OssianConfig::Configuration>("ossian.mrxzh.com", 80, "/api/argument");
	app.AddService<OnlineDebug>(
		[](OnlineDebug& option)
		{
			option.Connect("http://ossian.mrxzh.com/logger");
			option.StartLogging("OnlineLog",
			                    "OssianLog",
			                    "A piece of log.");
		});

	app.AddService<ossian::CANManager>();
	app.AddService<ossian::UARTManager>();
	app.AddService<ossian::IOListener>();
	app.AddService<ossian::MotorManager>();

	app.AddService<IReferee, RefereeAllMessagesMt>(
		[](IReferee& option)
		{
			option.AddReferee("/dev/ttyUSB0");
		});
	app.AddService<IRemote, RemoteMt>(
		[](IRemote& option)
		{
			option.AddRemote("/dev/ttyTHS2");
		});
	app.AddService<ICapacitor, CapacitorMt>(
		[](ICapacitor& option)
		{
			option.AddCapacitor("can0", 0x211, 0x210);
		});
	app.AddService<IGyro, GyroMt>(
		[](IGyro& option)
		{
			option.AddGyro("/dev/ttyUSB1");
		});

	app.AddService<Chassis>(
		[](Chassis& option)
		{
			option.AddMotor(Chassis::MotorPosition::LF, "can1", 1, 0x200);
			option.AddMotor(Chassis::MotorPosition::LR, "can1", 2, 0x200);
			option.AddMotor(Chassis::MotorPosition::RR, "can1", 3, 0x200);
			option.AddMotor(Chassis::MotorPosition::RF, "can1", 4, 0x200);
		});
	app.AddService<Gimbal>(
		[](Gimbal& option)
		{
			option.AddMotor(Gimbal::MotorPosition::Pitch, "can1", 7, 0x2ff);
			option.AddMotor(Gimbal::MotorPosition::Yaw, "can1", 6, 0x2ff);
		});
	//app.AddService<Aimbot>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
	app.AddExecutable<IOWorker>();
	app.AddExecutable<IOPeeker<0>>();
	app.AddExecutable<IOPeeker<1>>();
	
	//app.AddService<ossian::IExecutable, CameraPeeker>();
}
