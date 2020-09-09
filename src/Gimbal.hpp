﻿#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/motors/Motor.hpp>
#include <ossian/motors/DJIMotor.hpp>
#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "GyroA204.hpp"
#include "Aimbot.hpp"

#include <chrono>
#include <array>
#include <memory>
#include <atomic>

//云台电机数量
constexpr size_t kNumGimbalMotors = 2;

OSSIAN_MULTIPLE_MOTORS_STATUS(GimbalMotorsModel, kNumGimbalMotors);

class Gimbal : public ossian::IODataBuilder<std::mutex, GimbalMotorsModel>
{
public:
	enum MotorPosition
	{
		Pitch=0, Yaw
	};

	enum GimbalInputSrc
	{
		Disable, Init, RC, Mouse, Aimbot, Windmill
	};

	OSSIAN_SERVICE_SETUP(Gimbal(
		ossian::MotorManager* motorManager,
		ossian::IOData<GimbalMotorsModel>* ioData))
		: m_MotorManager(motorManager)
		, m_MotorsStatus()
		, m_IOData(ioData)
	{
		m_MotorMsgCheck.fill(false);
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

	auto MotorReceiveProc(const std::shared_ptr<ossian::DJIMotor6020Mt>& motor, MotorPosition position)->void
	{
		const auto status = motor->GetRef();
		motor->Lock();
		m_MotorsStatus.m_RPM[position] = status.m_RPM;
		m_MotorsStatus.m_Encoding[position] = status.m_Encoding;
		motor->UnLock();

		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[Pitch] && m_MotorMsgCheck[Yaw]))
		{
			return;
		}
		m_IOData->Set(m_MotorsStatus);

		m_MotorMsgCheck.fill(false);
	}

	void SendVoltageToMotors(const std::array<double, kNumGimbalMotors>& voltageSend)
	{
		for (size_t i = 0; i < kNumGimbalMotors; ++i)
		{
			m_Motors[i]->SetVoltage(voltageSend[i]);
		}
		m_Motors[Pitch]->Writer()->PackAndSend();
	}

private:
	ossian::MotorManager* m_MotorManager;  	
	std::array<std::shared_ptr<ossian::DJIMotor6020Mt>, kNumGimbalMotors> m_Motors;
	GimbalMotorsModel m_MotorsStatus;
	ossian::IOData<GimbalMotorsModel>* m_IOData;

	std::array<bool, kNumGimbalMotors> m_MotorMsgCheck;
};

class GimbalCtrlTask : public ossian::IExecutable
{
public:
	static constexpr double kMotorEcdToRadCoef = 2 * M_PI / 8192.0;
	static constexpr double kDegreeToRadCoef = M_PI / 180.0;

	//云台特殊位置 [TODO]在disable模式下，debug出限位和中值
	static constexpr uint16_t kPitchEcdLimit0 = 6885;
	static constexpr uint16_t kPitchEcdLimit1 = 226;
	static constexpr uint16_t kPitchEcdMid = 7517;

	static constexpr uint16_t kYawEcdLimit0 = 1789;
	static constexpr uint16_t kYawEcdLimit1 = 6273;
	static constexpr uint16_t kYawEcdMid = 4082;

	//最大最小的 相对（中值的）角度
	//确保 kMinRelativeAngle < kMaxRelativeAngle，包括符号
	const std::array<double, 2> kMaxRelativeAngle = { std::max(RelativeEcdToRad(kPitchEcdLimit0, kPitchEcdMid),
															   RelativeEcdToRad(kPitchEcdLimit1, kPitchEcdMid)),
													  std::max(RelativeEcdToRad(kYawEcdLimit0, kYawEcdMid),
														       RelativeEcdToRad(kYawEcdLimit1, kYawEcdMid)) };
	const std::array<double, 2> kMinRelativeAngle = { std::min(RelativeEcdToRad(kPitchEcdLimit0, kPitchEcdMid),
															   RelativeEcdToRad(kPitchEcdLimit1, kPitchEcdMid)),
													  std::min(RelativeEcdToRad(kYawEcdLimit0, kYawEcdMid),
															   RelativeEcdToRad(kYawEcdLimit1, kYawEcdMid)) };
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

	static constexpr double kYawRCSen = -0.00005; //-0.000005
	static constexpr double kPitchRCSen = 0.000006; //0.005

	//pid参数
	static std::array<double, 5> PIDAngleEcdPitchParams;
	static std::array<double, 5> PIDAngleGyroPitchParams;
	static std::array<double, 5> PIDAngleSpeedPitchParams;
	static std::array<double, 5> PIDAngleEcdYawParams;
	static std::array<double, 5> PIDAngleGyroYawParams;
	static std::array<double, 5> PIDAngleSpeedYawParams;
	static std::array<double, 5> PIDAutoAimInputParams;

	static constexpr double kGimbalCtrlPeriod = 0.005;
	static double kAngleSpeedFilterCoef;

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

	OSSIAN_SERVICE_SETUP(GimbalCtrlTask(ossian::IOData<GimbalMotorsModel>* motors,
		ossian::IOData<RemoteStatus>* remote,
		Gimbal* gimbal,
		Utils::ConfigLoader* config,
		ossian::IOData<GyroA204Status<GyroType::Pitch>>* gyroPitchListener,
		ossian::IOData<GyroA204Status<GyroType::Yaw>>* gyroYawListener,
		ossian::IOData<AutoAimData>* autoAimDataListener))
		: m_MotorsListener(motors)
		, m_RCListener(remote)
		, m_Gimbal(gimbal)
		, m_Config(config)
		, m_GyroA204PitchListener(gyroPitchListener)
		, m_GyroA204YawListener(gyroYawListener)
		, m_AutoAimDataListener(autoAimDataListener)
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

		PIDAutoAimInputParams[0] = m_Config->Instance<Configuration>()->mutable_pidautoaiminput()->kp();
		PIDAutoAimInputParams[1] = m_Config->Instance<Configuration>()->mutable_pidautoaiminput()->ki();
		PIDAutoAimInputParams[2] = m_Config->Instance<Configuration>()->mutable_pidautoaiminput()->kd();
		PIDAutoAimInputParams[3] = m_Config->Instance<Configuration>()->mutable_pidautoaiminput()->thout();
		PIDAutoAimInputParams[4] = m_Config->Instance<Configuration>()->mutable_pidautoaiminput()->thiout();

		kAngleSpeedFilterCoef = m_Config->Instance<Configuration>()->mutable_gimbal()->kanglespeedfiltercoef();

		m_GimbalCtrlSrc = Init;
		m_FlagInitGimbal = true;
		m_FlagInitGyro = true;

		m_PIDAngleEcd[Pitch].SetParams(PIDAngleEcdPitchParams);
		m_PIDAngleEcd[Pitch].SetFlagAngleLoop();

		m_PIDAngleGyro[Pitch].SetParams(PIDAngleGyroPitchParams);
		m_PIDAngleGyro[Pitch].SetFlagAngleLoop();

		m_PIDAngleSpeed[Pitch].SetParams(PIDAngleSpeedPitchParams);

		m_PIDAutoAimInput[Pitch].SetParams(PIDAutoAimInputParams);
		m_PIDAutoAimInput[Pitch].SetFlagAngleLoop();

		m_PIDAngleEcd[Yaw].SetParams(PIDAngleEcdYawParams);
		m_PIDAngleEcd[Yaw].SetFlagAngleLoop();

		m_PIDAngleGyro[Yaw].SetParams(PIDAngleGyroYawParams);
		m_PIDAngleGyro[Yaw].SetFlagAngleLoop();

		m_PIDAngleSpeed[Yaw].SetParams(PIDAngleSpeedYawParams);

		m_PIDAutoAimInput[Yaw].SetParams(PIDAutoAimInputParams);
		m_PIDAutoAimInput[Yaw].SetFlagAngleLoop();

		FirstOrderFilter ecdAngleFilter(0.25, kGimbalCtrlPeriod);
		m_EcdAngleFilters.fill(ecdAngleFilter);

		FirstOrderFilter gyroFilter(0.0001, kGimbalCtrlPeriod);
		m_GyroSpeedFilters.fill(gyroFilter);

		m_LastEcdTimeStamp.fill(std::chrono::high_resolution_clock::time_point());
		m_VoltageSend.fill(0);

		m_AutoAimPredictor.SetMatsForAutoAim(3);

		/*m_GyroA204YawListener->AddOnChange([](const GyroA204Status<GyroType::Yaw>& value) {
			SPDLOG_INFO("@ImuYaw=[$ZAngleYaw={},$ZSpeedYaw={}]",
				value.m_ZAxisAngle, value.m_ZAxisSpeed); });
		m_GyroA204PitchListener->AddOnChange([](const GyroA204Status<GyroType::Pitch>& value) {
			SPDLOG_INFO("@ImuPitch=[$ZAnglePitch={},$ZSpeedPitch={}]",
				value.m_ZAxisAngle, value.m_ZAxisSpeed); });*/
		/*m_GyroListener->AddOnChange([](const GyroModel& value) {
			SPDLOG_INFO("@GyroOnChange=[$roll={},$pitch={},$yaw={}]",
				value.m_Wx, value.m_Wy, value.m_Wz); });*/

		//m_VoltageSetFilters[Pitch].SetState(0.09, 0.002);
		//m_VoltageSetFilters[Yaw].SetState(0.09, 0.002);
	}

	GimbalInputSrc GimbalCtrlSrc() { return m_GimbalCtrlSrc.load(); }

	double RelativeAngleToChassis()
	{
		return RelativeEcdToRad(m_EcdYaw.load(), kYawEcdMid); //[TODO]负号？
	}

	void UpdateGimbalSensorFeedback()
	{
		m_GimbalSensorValues.rc = m_RCListener->Get();
		m_GimbalSensorValues.relativeAngle[Pitch] = RelativeEcdToRad(m_MotorsStatus.m_Encoding[Pitch], kPitchEcdMid);
		m_GimbalSensorValues.relativeAngle[Yaw] = RelativeEcdToRad(m_MotorsStatus.m_Encoding[Yaw], kYawEcdMid);

		m_GimbalSensorValues.imuPitch = m_GyroA204PitchListener->Get();
		m_GimbalSensorValues.imuPitch.m_ZAxisSpeed *= kDegreeToRadCoef;
		m_GimbalSensorValues.imuPitch.m_ZAxisAngle *= kDegreeToRadCoef;
		/*if(!m_FlagInitGimbal)
			m_GimbalSensorValues.imuPitch.m_ZAxisAngle = ClampLoop(static_cast<double>(m_GimbalSensorValues.imuPitch.m_ZAxisAngle), m_GyroAngleZeroPoints[Pitch], m_GyroAngleZeroPoints[Pitch] + M_PI * 2);*/

		m_GimbalSensorValues.imuYaw = m_GyroA204YawListener->Get();
		m_GimbalSensorValues.imuYaw.m_ZAxisSpeed *= -kDegreeToRadCoef;
		m_GimbalSensorValues.imuYaw.m_ZAxisAngle *= -kDegreeToRadCoef;
		m_GimbalSensorValues.imuYaw.m_ZAxisSpeed = DeadbandLimit(m_GimbalSensorValues.imuYaw.m_ZAxisSpeed, 0.003f);

		auto tempAutoAimData = m_AutoAimDataListener->Get();
		if (m_GimbalSensorValues.autoAimData.m_Timestamp != tempAutoAimData.m_Timestamp)
			m_GimbalSensorValues.autoAimData = tempAutoAimData;
		else
		{
			m_GimbalSensorValues.autoAimData.m_Found = false;
			m_GimbalSensorValues.autoAimData.m_Pitch = 0;
			m_GimbalSensorValues.autoAimData.m_Yaw = 0;
			m_GimbalSensorValues.autoAimData.m_Dist = 0;
		}
		//m_GimbalSensorValues.autoAimData = m_AutoAimDataListener->Get();
		//std::cerr << m_GimbalSensorValues.autoAimData.m_Pitch << '\t' << m_GimbalSensorValues.autoAimData.m_Yaw << '\t' << m_GimbalSensorValues.autoAimData.m_Dist << std::endl;
		/*if(!m_FlagInitGimbal)
			m_GimbalSensorValues.imuYaw.m_ZAxisAngle = ClampLoop(static_cast<double>(m_GimbalSensorValues.imuYaw.m_ZAxisAngle), m_GyroAngleZeroPoints[Yaw], m_GyroAngleZeroPoints[Yaw] + M_PI * 2);*/
		
		//std::swap(m_GimbalSensorValues.imu.m_Roll, m_GimbalSensorValues.imu.m_Pitch);
		//std::swap(m_GimbalSensorValues.imu.m_Wx, m_GimbalSensorValues.imu.m_Wy);
		//m_GimbalSensorValues.imu.m_Pitch = -m_GimbalSensorValues.imu.m_Pitch;
		////gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		//m_GimbalSensorValues.imu.m_Wz = cos(m_GimbalSensorValues.relativeAngle[Pitch]) * m_GimbalSensorValues.imu.m_Wz
		//	- sin(m_GimbalSensorValues.relativeAngle[Pitch]) * m_GimbalSensorValues.imu.m_Wx;

		/*SPDLOG_INFO("@IMUAngle=[$GPitch={},$GYaw={}]",
			m_GimbalSensorValues.imuPitch.m_ZAxisAngle,
			m_GimbalSensorValues.imuYaw.m_ZAxisAngle);
		SPDLOG_INFO("@IMUSpeed=[$WPitch={},$WYaw={}]",
			m_GimbalSensorValues.imuPitch.m_ZAxisSpeed,
			m_GimbalSensorValues.imuYaw.m_ZAxisSpeed);*/
		/*SPDLOG_DEBUG("@IMUMagnetometer=[$roll_h={},$pitch_h={},$yaw_h={}]",
			m_GimbalSensorValues.imu.m_Hx,
			m_GimbalSensorValues.imu.m_Hy,
			m_GimbalSensorValues.imu.m_Hz);*/

		/*SPDLOG_INFO("@MotorEncoder=[$EPitch={},$EYaw={}]",
			m_MotorsStatus.m_Encoding[Pitch],
			m_MotorsStatus.m_Encoding[Yaw]);*/

	}


	//设置云台角度输入来源
	void GimbalCtrlSrcSet();

	//获得操作手期望的角度输入
	void GimbalCtrlInputProc();

	//根据遥控数据，设置角度期望值rad
	void GimbalExpAngleSet(MotorPosition position);

	//双环pid计算 
	void GimbalCtrl(MotorPosition position);

	auto ExecuteProc() -> void override
	{
		using Clock = std::chrono::high_resolution_clock;
		using TimeStamp = Clock::time_point;

		TimeStamp lastTime = Clock::now();

		while (true)
		{
			while (5000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();

			/*SPDLOG_INFO("@Interval=[$t={}]",
				std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count() / 1000.0);*/

			m_MotorsStatus = m_MotorsListener->Get();
			m_EcdYaw = m_MotorsStatus.m_Encoding[Yaw];

			UpdateGimbalSensorFeedback();

			GimbalCtrlSrcSet();
			GimbalCtrlInputProc();
			//[TODO] 模式切换过渡

			GimbalExpAngleSet(Pitch);
			GimbalExpAngleSet(Yaw);

			GimbalCtrl(Pitch);
			GimbalCtrl(Yaw);
		}
	}

private:
	ossian::IOData<GimbalMotorsModel>* m_MotorsListener;
	Utils::ConfigLoader* m_Config;
	Gimbal* m_Gimbal;
	ossian::IOData<RemoteStatus>* m_RCListener;  //遥控器
	ossian::IOData<GyroA204Status<GyroType::Pitch>>* m_GyroA204PitchListener;
	ossian::IOData<GyroA204Status<GyroType::Yaw>>* m_GyroA204YawListener;
	ossian::IOData<AutoAimData>* m_AutoAimDataListener;

	std::array<GimbalAngleMode, 2> m_CurGimbalAngleMode;
	std::atomic<GimbalInputSrc> m_GimbalCtrlSrc;
	GimbalMotorsModel m_MotorsStatus;
	std::atomic<uint16_t> m_EcdYaw{};

	struct GimbalSensorFeedback
	{
		double relativeAngle[kNumGimbalMotors];
		RemoteStatus rc;	 //遥控器数据
		GyroA204Status<GyroType::Pitch> imuPitch;
		GyroA204Status<GyroType::Yaw> imuYaw;
		AutoAimData autoAimData;
		//double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //云台imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
	} m_GimbalSensorValues;

	bool m_FlagInitGimbal, m_FlagInitGyro; //上电后陀螺仪静置5秒用于自校正
	std::chrono::high_resolution_clock::time_point m_TimestampInit;

	std::array<double, kNumGimbalMotors> m_AngleInput;
	std::array<double, kNumGimbalMotors> m_LastEcdAngle;
	std::array<std::chrono::high_resolution_clock::time_point, kNumGimbalMotors> m_LastEcdTimeStamp;

	std::array<FirstOrderFilter, kNumGimbalMotors> m_EcdAngleFilters;
	std::array<FirstOrderFilter, kNumGimbalMotors> m_GyroSpeedFilters; //陀螺仪角速度滤波，去除摩擦轮共振干扰
	/*double m_YawAdd, m_PitchAdd; //角度增量rad
	double m_LastYaw, m_LastPitch;*/
	std::array<double, kNumGimbalMotors> m_GyroAngleSet; //累加 陀螺仪模式
	std::array<double, kNumGimbalMotors> m_EcdAngleSet; //累加 编码器模式

	//设定角度--->旋转角速度  旋转角速度-->6020控制电压
	std::array<PIDController, kNumGimbalMotors> m_PIDAutoAimInput;
	std::array<PIDController, kNumGimbalMotors> m_PIDAngleEcd, m_PIDAngleGyro, m_PIDAngleSpeed;
	std::array<double, kNumGimbalMotors> m_VoltageSend;

	KalmanFilter m_AutoAimPredictor{ 4,2,0 };
	//std::array<FirstOrderFilter, kNumGimbalMotors> m_VoltageSetFilters;
};


#endif // OSSIAN_GIMBAL_HPP