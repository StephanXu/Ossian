#include <ossian/ossian.hpp>
#include <mimalloc.h>

#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Gun.hpp"
#include "OnlineDebug.hpp"
#include "Remote.hpp"
#include "Capacitor.hpp"
#include "Referee.hpp"
#include "Phototube.hpp"
#include "Gyro.hpp"
#include "GyroA204.hpp"
#include "TOFNooploop.hpp"
#include "IOPeeker.hpp"
#include "CameraPeeker.hpp"

#include <Config.schema.hpp>
#include <LaunchSettings.schema.hpp>

Startup::Startup()
{
	// Initialize logger
	const auto console = spdlog::stderr_color_mt("console");
	spdlog::set_default_logger(console);
	spdlog::set_pattern("[%Y-%m-%dT%T.%e%z] [%-5t] %^[%l]%$ %v");
	spdlog::set_level(spdlog::level::trace);

	SPDLOG_INFO("MI_VERSION: {}", mi_version());

	// Load configuration
	m_AppConfig.LoadConfigFromFile("AppSettings.json");
}

void Startup::ConfigServices(AppBuilder& app)
{
	app.AddService<Utils::ConfigLoader<Config::ConfigSchema>>()
	   .LoadFromUrl(*m_AppConfig->ossian->onlineArguments->server,
	                *m_AppConfig->ossian->onlineArguments->argId);
	app.AddService<OnlineDebug>(
		[config = m_AppConfig](OnlineDebug& option)
		{
			const auto logConfig = config->ossian->onlineDebug;
			option.Connect(*logConfig->loggerUrl);
			option.StartLogging("OnlineLog",
			                    *logConfig->logName,
			                    *logConfig->logDesc,
			                    *config->ossian->onlineArguments->argId);
		});

	app.AddService<ossian::CANManager>();
	app.AddService<ossian::UARTManager>();
	app.AddService<ossian::IOListener>();
	app.AddService<ossian::MotorManager>();

	app.AddService<IReferee, RefereeAllMessagesMt>(
		[](IReferee& option)
		{
			option.AddReferee("/dev/ttyTHS2");
		});
	app.AddService<RemoteMt>(
		[](RemoteMt& option)
		{
			option.Add("/dev/ttyUSB0",
			           100000,
			           ossian::UARTProperties::FlowControlNone,
			           ossian::UARTProperties::DataBits8,
			           ossian::UARTProperties::StopBits1,
			           ossian::UARTProperties::ParityEven);
		});
	app.AddService<ICapacitor, CapacitorMt>(
		[](ICapacitor& option)
		{
			//option.AddCapacitor("can0", 0x211, 0x210);
		});
	app.AddService<IGyro, GyroMt>(
		[](IGyro& option)
		{
			//option.AddGyro("/dev/ttyUSB0");
		});
	app.AddService<GyroA204St<GyroType::Yaw>>(
		[](GyroA204St<GyroType::Yaw>& option)
		{
			option.Add("can0", 0x402, 0x403);
		});
	app.AddService<GyroA204St<GyroType::Pitch>>(
		[](GyroA204St<GyroType::Pitch>& option)
		{
			option.Add("can1", 0x402, 0x403);
		});
	app.AddService<PhototubeMt>(
		[](PhototubeMt& option)
		{
			option.Add("can0", 0x300);
		});
	app.AddService<Chassis>(
		[](Chassis& option)
		{
			option.AddMotor(Chassis::MotorPosition::LF, "can0", 1, 0x200);
			option.AddMotor(Chassis::MotorPosition::LR, "can0", 2, 0x200);
			option.AddMotor(Chassis::MotorPosition::RR, "can0", 3, 0x200);
			option.AddMotor(Chassis::MotorPosition::RF, "can0", 4, 0x200);
		});
	app.AddService<Gimbal>(
		[](Gimbal& option)
		{
			option.AddMotor(Gimbal::MotorPosition::Pitch, "can1", 7, 0x2ff);
			option.AddMotor(Gimbal::MotorPosition::Yaw, "can1", 6, 0x2ff);
		});
	app.AddService<Gun>(
		[](Gun& option)
		{
			option.AddMotor(Gun::MotorPosition::FricBelow, "can1", 2, 0x200);
			option.AddMotor(Gun::MotorPosition::FricUpper, "can1", 1, 0x200);
			option.AddMotor(Gun::MotorPosition::Feed, "can1", 3, 0x200);
		});
	app.AddService<Aimbot>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
	app.AddExecutable<IOWorker>();
	app.AddExecutable<IOPeeker<0>>();
	app.AddExecutable<IOPeeker<1>>();

	app.AddExecutable<ChassisCtrlTask>();
	app.AddExecutable<GimbalCtrlTask>();
	app.AddExecutable<FricCtrlTask>();
	app.AddExecutable<FeedCtrlTask>();
	app.AddExecutable<CameraPeeker>();
}
