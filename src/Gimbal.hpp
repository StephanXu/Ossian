#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/motors/Motor.hpp>
#include <ossian/motors/DJIMotor.hpp>
#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gyro.hpp"

#include <chrono>
#include <array>
#include <memory>
#include <atomic>

using hrClock = std::chrono::high_resolution_clock;
class Gimbal
{
public:
	static constexpr double kMotorEcdToRadCoef = 2 * M_PI / 8192;
	//云台特殊位置 [TODO]在disable模式下，debug出限位和中值
	static constexpr uint16_t kPitchMinEcd = 4176;
	static constexpr uint16_t kPitchMaxEcd = 5670;
	static constexpr uint16_t kPitchMidEcd = 4772;
	
	static constexpr uint16_t kYawMinEcd = 7552;
	static constexpr uint16_t kYawMaxEcd = 3456;
	static constexpr uint16_t kYawMidEcd = 5504;

	//最大最小的 相对（中值的）角度
	const std::array<double, 2> kMaxRelativeAngle = { RelativeEcdToRad(kPitchMaxEcd, kPitchMidEcd),
													  RelativeEcdToRad(kYawMaxEcd, kYawMidEcd) };
	const std::array<double, 2> kMinRelativeAngle = { RelativeEcdToRad(kPitchMinEcd, kPitchMidEcd),
													  RelativeEcdToRad(kYawMinEcd, kYawMidEcd) };
	/*
	static constexpr double   PITCH_MAX_RAD = kPitchMaxEcd * kMotorEcdToRadCoef;
	static constexpr double   PITCH_MIN_RAD = kPitchMinEcd * kMotorEcdToRadCoef;
	static constexpr double   YAW_MAX_RAD   = kYawMaxEcd   * kMotorEcdToRadCoef;
	static constexpr double   YAW_MIN_RAD   = kYawMinEcd   * kMotorEcdToRadCoef;
	*/

	//遥控器解析
	static constexpr int16_t kGimbalRCDeadband = 10; //摇杆死区
	static constexpr size_t kYawChannel = 2;
	static constexpr size_t kPitchChannel = 3;
	static constexpr size_t kGimbalModeChannel = 1; //选择云台状态的开关通道

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	static constexpr double kYawRCSen= -0.000005;
	static constexpr double kPitchRCSen = -0.000006; //0.005

	//pid参数
	static std::array<double, 5> PIDAngleEcdPitchParams;
	static std::array<double, 5> PIDAngleGyroPitchParams;
	static std::array<double, 5> PIDAngleSpeedPitchParams;
	static std::array<double, 5> PIDAngleEcdYawParams;
	static std::array<double, 5> PIDAngleGyroYawParams;
	static std::array<double, 5> PIDAngleSpeedYawParams;
	
	enum GimbalAngleMode
	{
		Gyro, Encoding
	};

	enum GimbalInputSrc
	{
		Disable, Init, RC, Mouse, Aimbot, Windmill
	};

	enum MotorPosition
	{
		Pitch, Yaw
	};
	
	OSSIAN_SERVICE_SETUP(Gimbal(ossian::MotorManager* motorManager, 
								ossian::IOData<RemoteStatus>* remote, 
								Utils::ConfigLoader* config,
								ossian::IOData<GyroModel>* gyroListener))
		: m_MotorManager(motorManager)
		, m_RC(remote)
		, m_Config(config)
		, m_GyroListener(gyroListener)
	{
		using OssianConfig::Configuration;
		PIDAngleEcdPitchParams[0] = m_Config->Instance<Configuration>()->mutable_pidangleecdpitch()->kp();
		PIDAngleEcdPitchParams[1] = m_Config->Instance<Configuration>()->mutable_pidangleecdpitch()->ki();
		PIDAngleEcdPitchParams[2] = m_Config->Instance<Configuration>()->mutable_pidangleecdpitch()->kd();
		PIDAngleEcdPitchParams[3] = m_Config->Instance<Configuration>()->mutable_pidangleecdpitch()->thout();
		PIDAngleEcdPitchParams[4] = m_Config->Instance<Configuration>()->mutable_pidangleecdpitch()->thiout();

		PIDAngleGyroPitchParams[0] = m_Config->Instance<Configuration>()->mutable_pidanglegyropitch()->kp();
		PIDAngleGyroPitchParams[1] = m_Config->Instance<Configuration>()->mutable_pidanglegyropitch()->ki();
		PIDAngleGyroPitchParams[2] = m_Config->Instance<Configuration>()->mutable_pidanglegyropitch()->kd();
		PIDAngleGyroPitchParams[3] = m_Config->Instance<Configuration>()->mutable_pidanglegyropitch()->thout();
		PIDAngleGyroPitchParams[4] = m_Config->Instance<Configuration>()->mutable_pidanglegyropitch()->thiout();

		PIDAngleSpeedPitchParams[0] = m_Config->Instance<Configuration>()->mutable_pidanglespeedpitch()->kp();
		PIDAngleSpeedPitchParams[1] = m_Config->Instance<Configuration>()->mutable_pidanglespeedpitch()->ki();
		PIDAngleSpeedPitchParams[2] = m_Config->Instance<Configuration>()->mutable_pidanglespeedpitch()->kd();
		PIDAngleSpeedPitchParams[3] = m_Config->Instance<Configuration>()->mutable_pidanglespeedpitch()->thout();
		PIDAngleSpeedPitchParams[4] = m_Config->Instance<Configuration>()->mutable_pidanglespeedpitch()->thiout();

		PIDAngleEcdYawParams[0] = m_Config->Instance<Configuration>()->mutable_pidangleecdyaw()->kp();
		PIDAngleEcdYawParams[1] = m_Config->Instance<Configuration>()->mutable_pidangleecdyaw()->ki();
		PIDAngleEcdYawParams[2] = m_Config->Instance<Configuration>()->mutable_pidangleecdyaw()->kd();
		PIDAngleEcdYawParams[3] = m_Config->Instance<Configuration>()->mutable_pidangleecdyaw()->thout();
		PIDAngleEcdYawParams[4] = m_Config->Instance<Configuration>()->mutable_pidangleecdyaw()->thiout();

		PIDAngleGyroYawParams[0] = m_Config->Instance<Configuration>()->mutable_pidanglegyroyaw()->kp();
		PIDAngleGyroYawParams[1] = m_Config->Instance<Configuration>()->mutable_pidanglegyroyaw()->ki();
		PIDAngleGyroYawParams[2] = m_Config->Instance<Configuration>()->mutable_pidanglegyroyaw()->kd();
		PIDAngleGyroYawParams[3] = m_Config->Instance<Configuration>()->mutable_pidanglegyroyaw()->thout();
		PIDAngleGyroYawParams[4] = m_Config->Instance<Configuration>()->mutable_pidanglegyroyaw()->thiout();

		PIDAngleSpeedYawParams[0] = m_Config->Instance<Configuration>()->mutable_pidanglespeedyaw()->kp();
		PIDAngleSpeedYawParams[1] = m_Config->Instance<Configuration>()->mutable_pidanglespeedyaw()->ki();
		PIDAngleSpeedYawParams[2] = m_Config->Instance<Configuration>()->mutable_pidanglespeedyaw()->kd();
		PIDAngleSpeedYawParams[3] = m_Config->Instance<Configuration>()->mutable_pidanglespeedyaw()->thout();
		PIDAngleSpeedYawParams[4] = m_Config->Instance<Configuration>()->mutable_pidanglespeedyaw()->thiout();

		m_GimbalCtrlSrc = RC;
		m_FlagInitGimbal = true;
		m_MotorMsgCheck.fill(false);

		m_PIDAngleEcd[Pitch].SetParams(PIDAngleEcdPitchParams);
		m_PIDAngleGyro[Pitch].SetParams(PIDAngleGyroPitchParams);
		m_PIDAngleSpeed[Pitch].SetParams(PIDAngleSpeedPitchParams);
		m_PIDAngleEcd[Yaw].SetParams(PIDAngleEcdYawParams);
		m_PIDAngleGyro[Yaw].SetParams(PIDAngleGyroYawParams);
		m_PIDAngleSpeed[Yaw].SetParams(PIDAngleSpeedYawParams);

		m_LastEcdTimeStamp.fill(hrClock::time_point());
		m_CurrentSend.fill(0);
		
		/*m_GyroListener->AddOnChange([](const GyroModel& value) {
			SPDLOG_INFO("@GyroOnChange=[$gyro={}]",1);});*/

	}

	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int motorId,
				  const unsigned int writerCanId)->void
	{
		m_Motors[position] =
			m_MotorManager->AddMotor<ossian::DJIMotor6020Mt>(
				location,
				m_MotorManager->GetOrAddWriter<ossian::DJIMotor6020WriterMt>(location, writerCanId),
				[this, position](const std::shared_ptr<ossian::DJIMotor6020Mt>& motor)
				{
					MotorReceiveProc(motor, position);
				},
				motorId);
	}

	double RelativeAngleToChassis() { return -RelativeEcdToRad(m_YawEcd.load(), kYawMidEcd); } //[TODO]负号？

	GimbalInputSrc GimbalCtrlSrc() { return m_GimbalCtrlSrc.load(); }

	void UpdateGimbalSensorFeedback()
	{
		m_YawEcd = m_Motors[Yaw]->Get().m_Encoding;
		m_GimbalSensorValues.rc = m_RC->Get();
		m_GimbalSensorValues.imu = m_GyroListener->Get();
		std::swap(m_GimbalSensorValues.imu.m_Roll, m_GimbalSensorValues.imu.m_Pitch);
		std::swap(m_GimbalSensorValues.imu.m_Wx, m_GimbalSensorValues.imu.m_Wy);
		m_GimbalSensorValues.imu.m_Pitch = -m_GimbalSensorValues.imu.m_Pitch;
		//gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		m_GimbalSensorValues.imu.m_Wz = cos(m_GimbalSensorValues.imu.m_Pitch) * m_GimbalSensorValues.imu.m_Wz 
			- sin(m_GimbalSensorValues.imu.m_Pitch) * m_GimbalSensorValues.imu.m_Wx;
		
		SPDLOG_DEBUG("@IMUAngle=[$roll_gyro={},$pitch_gyro={},$yaw_gyro={}]",
			m_GimbalSensorValues.imu.m_Roll,
			m_GimbalSensorValues.imu.m_Pitch,
			m_GimbalSensorValues.imu.m_Yaw);
		SPDLOG_DEBUG("@IMUSpeed=[$roll_w={},$pitch_w={},$yaw_w={}]",
			m_GimbalSensorValues.imu.m_Wx,
			m_GimbalSensorValues.imu.m_Wy,
			m_GimbalSensorValues.imu.m_Wz);
		SPDLOG_DEBUG("@IMUMagnetometer=[$roll_h={},$pitch_h={},$yaw_h={}]",
			m_GimbalSensorValues.imu.m_Hx,
			m_GimbalSensorValues.imu.m_Hy,
			m_GimbalSensorValues.imu.m_Hz);

		SPDLOG_DEBUG("@MotorEncoder=[$pitch_ecd={},$yaw_ecd={}]",
			m_Motors[Pitch]->Get().m_Encoding,
			m_Motors[Yaw]->Get().m_Encoding);
		
	}
	//设置云台角度输入来源
	void GimbalCtrlSrcSet();

	//获得操作手期望的角度输入
	void GimbalCtrlInputProc();

	//根据遥控数据，设置角度期望值rad
	void GimbalExpAngleSet(MotorPosition position);

	//双环pid计算 
	void GimbalCtrl(MotorPosition position);

	auto MotorReceiveProc(const std::shared_ptr<ossian::DJIMotor6020Mt>& motor, MotorPosition position)->void
	{
		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[Pitch] && m_MotorMsgCheck[Yaw]))
			return;

		UpdateGimbalSensorFeedback();

		GimbalCtrlSrcSet();
		GimbalCtrlInputProc();
		//[TODO] 模式切换过渡

		GimbalExpAngleSet(Pitch);
		GimbalExpAngleSet(Yaw);

		GimbalCtrl(Pitch);
		GimbalCtrl(Yaw);

		m_MotorMsgCheck.fill(false);
	}


private:
	ossian::MotorManager* m_MotorManager;  	
	std::array<std::shared_ptr<ossian::DJIMotor6020Mt>, 2> m_Motors;  	
	hrClock::time_point m_LastRefresh;
	Utils::ConfigLoader* m_Config;
	ossian::IOData<RemoteStatus>* m_RC;  //遥控器
	ossian::IOData<GyroModel>* m_GyroListener;

	GimbalAngleMode m_CurGimbalAngleMode, m_LastGimbalAngleMode;
	std::atomic<GimbalInputSrc> m_GimbalCtrlSrc;
	std::array<bool, 2> m_MotorMsgCheck;
	struct GimbalSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		GyroModel imu;
		//double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //云台imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
	} m_GimbalSensorValues;
	std::atomic<uint16_t> m_YawEcd;

	bool m_FlagInitGimbal;

	std::array<double, 2> m_AngleInput;
	std::array<double, 2> m_LastEcdAngle;
	std::array<hrClock::time_point, 2> m_LastEcdTimeStamp;

	/*double m_YawAdd, m_PitchAdd; //角度增量rad
	double m_LastYaw, m_LastPitch;*/
	std::array<double, 2> m_GyroAngleSet; //累加 陀螺仪模式
	std::array<double, 2> m_EcdAngleSet; //累加 编码器模式

	//设定角度--->旋转角速度  旋转角速度-->6020控制电压
	std::array<PIDController,2> m_PIDAngleEcd, m_PIDAngleGyro, m_PIDAngleSpeed; 
	std::array<double, 2> m_CurrentSend;
};

#endif // OSSIAN_GIMBAL_HPP