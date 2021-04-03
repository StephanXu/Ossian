#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/motors/Motor.hpp>
#include <ossian/motors/DJIMotor.hpp>
#include <ossian/IOData.hpp>
#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "GyroA110.hpp"
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

	enum GimbalCtrlMode
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

enum class GimbalCtrlMode
{
	Disable, Init, RC, Mouse, Aimbot, Windmill
};

struct GimbalStatus
{
	GimbalCtrlMode m_CtrlMode;
	double m_RelativeAngleToChassis; ///< 底盘坐标系与云台坐标系的夹角 当前yaw编码值减去中值 rad
	bool m_AutoAimShoot;
};
class GimbalCtrlTask : public ossian::IExecutable, public ossian::IODataBuilder<ossian::null_mutex, GimbalStatus>
{
public:
	static constexpr double kMotorEcdToRadCoef = 2 * M_PI / 8192.0;
	static constexpr double kDegreeToRadCoef = M_PI / 180.0;

	//云台特殊位置 [TODO]在disable模式下，debug出限位和中值
	static constexpr uint16_t kPitchEcdLimit0 = 6236;
	static constexpr uint16_t kPitchEcdLimit1 = 7754;
	static constexpr uint16_t kPitchEcdMid = 6906;

	static constexpr uint16_t kYawEcdLimit0 = 2925;
	static constexpr uint16_t kYawEcdLimit1 = 7019;
	static constexpr uint16_t kYawEcdMid = 852;

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

	static constexpr double kYawRCSen = -0.00005; //-0.000005
	static constexpr double kPitchRCSen = 0.00005; //0.005 0.00001

	static constexpr double kYawMouseSen = -0.00012;
	static constexpr double kPitchMouseSen = -0.0001;

	//pid参数
	static std::array<double, 5> PIDAngleEcdPitchParams;
	static std::array<double, 5> PIDAngleGyroPitchParams;
	static std::array<double, 5> PIDAngleGyroPitchAutoAimParams;
	static std::array<double, 5> PIDAngleSpeedPitchParams;
	static std::array<double, 5> PIDAngleEcdYawParams;
	static std::array<double, 5> PIDAngleGyroYawParams;
	static std::array<double, 5> PIDAngleGyroYawAutoAimParams;
	static std::array<double, 5> PIDAngleSpeedYawParams;
	static std::array<double, 5> PIDAutoAimInputParams;

	static constexpr double kGimbalCtrlPeriod = 0.006;
	//static double kAngleSpeedFilterCoef;

	enum GimbalAngleMode
	{
		Gyro, Encoding
	};

	enum MotorPosition
	{
		Pitch, Yaw
	};

	OSSIAN_SERVICE_SETUP(GimbalCtrlTask(ossian::IOData<GimbalMotorsModel>* motors,
		ossian::IOData<RemoteStatus>* remote,
		Gimbal* gimbal,
		ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		ossian::IOData<GyroA110Status<GyroType::Gimbal>>* gyroListener,
		ossian::IOData<AutoAimStatus>* autoAimStatusListener,
		ossian::IOData<GimbalStatus>* gimbalStatusSender))
		: m_MotorsListener(motors)
		, m_RCListener(remote)
		, m_Gimbal(gimbal)
		, m_Config(config)
		, m_GyroListener(gyroListener)
		, m_AutoAimStatusListener(autoAimStatusListener)
		, m_GimbalStatusSender(gimbalStatusSender)
	{
		PIDAngleEcdPitchParams[0] = *m_Config->Instance()->pids->pidAngleEcdPitch->kP;
		PIDAngleEcdPitchParams[1] = *m_Config->Instance()->pids->pidAngleEcdPitch->kI;
		PIDAngleEcdPitchParams[2] = *m_Config->Instance()->pids->pidAngleEcdPitch->kD;
		PIDAngleEcdPitchParams[3] = *m_Config->Instance()->pids->pidAngleEcdPitch->thOut;
		PIDAngleEcdPitchParams[4] = *m_Config->Instance()->pids->pidAngleEcdPitch->thIOut;

		PIDAngleGyroPitchParams[0] = *m_Config->Instance()->pids->pidAngleGyroPitch->kP;
		PIDAngleGyroPitchParams[1] = *m_Config->Instance()->pids->pidAngleGyroPitch->kI;
		PIDAngleGyroPitchParams[2] = *m_Config->Instance()->pids->pidAngleGyroPitch->kD;
		PIDAngleGyroPitchParams[3] = *m_Config->Instance()->pids->pidAngleGyroPitch->thOut;
		PIDAngleGyroPitchParams[4] = *m_Config->Instance()->pids->pidAngleGyroPitch->thIOut;

		PIDAngleGyroPitchAutoAimParams[0] = *m_Config->Instance()->pids->pidAngleGyroPitchAutoAim->kP;
		PIDAngleGyroPitchAutoAimParams[1] = *m_Config->Instance()->pids->pidAngleGyroPitchAutoAim->kI;
		PIDAngleGyroPitchAutoAimParams[2] = *m_Config->Instance()->pids->pidAngleGyroPitchAutoAim->kD;
		PIDAngleGyroPitchAutoAimParams[3] = *m_Config->Instance()->pids->pidAngleGyroPitchAutoAim->thOut;
		PIDAngleGyroPitchAutoAimParams[4] = *m_Config->Instance()->pids->pidAngleGyroPitchAutoAim->thIOut;

		PIDAngleSpeedPitchParams[0] = *m_Config->Instance()->pids->pidAngleSpeedPitch->kP;
		PIDAngleSpeedPitchParams[1] = *m_Config->Instance()->pids->pidAngleSpeedPitch->kI;
		PIDAngleSpeedPitchParams[2] = *m_Config->Instance()->pids->pidAngleSpeedPitch->kD;
		PIDAngleSpeedPitchParams[3] = *m_Config->Instance()->pids->pidAngleSpeedPitch->thOut;
		PIDAngleSpeedPitchParams[4] = *m_Config->Instance()->pids->pidAngleSpeedPitch->thIOut;

		PIDAngleEcdYawParams[0] = *m_Config->Instance()->pids->pidAngleEcdYaw->kP;
		PIDAngleEcdYawParams[1] = *m_Config->Instance()->pids->pidAngleEcdYaw->kI;
		PIDAngleEcdYawParams[2] = *m_Config->Instance()->pids->pidAngleEcdYaw->kD;
		PIDAngleEcdYawParams[3] = *m_Config->Instance()->pids->pidAngleEcdYaw->thOut;
		PIDAngleEcdYawParams[4] = *m_Config->Instance()->pids->pidAngleEcdYaw->thIOut;

		PIDAngleGyroYawParams[0] = *m_Config->Instance()->pids->pidAngleGyroYaw->kP;
		PIDAngleGyroYawParams[1] = *m_Config->Instance()->pids->pidAngleGyroYaw->kI;
		PIDAngleGyroYawParams[2] = *m_Config->Instance()->pids->pidAngleGyroYaw->kD;
		PIDAngleGyroYawParams[3] = *m_Config->Instance()->pids->pidAngleGyroYaw->thOut;
		PIDAngleGyroYawParams[4] = *m_Config->Instance()->pids->pidAngleGyroYaw->thIOut;

		PIDAngleGyroYawAutoAimParams[0] = *m_Config->Instance()->pids->pidAngleGyroYawAutoAim->kP;
		PIDAngleGyroYawAutoAimParams[1] = *m_Config->Instance()->pids->pidAngleGyroYawAutoAim->kI;
		PIDAngleGyroYawAutoAimParams[2] = *m_Config->Instance()->pids->pidAngleGyroYawAutoAim->kD;
		PIDAngleGyroYawAutoAimParams[3] = *m_Config->Instance()->pids->pidAngleGyroYawAutoAim->thOut;
		PIDAngleGyroYawAutoAimParams[4] = *m_Config->Instance()->pids->pidAngleGyroYawAutoAim->thIOut;

		PIDAngleSpeedYawParams[0] = *m_Config->Instance()->pids->pidAngleSpeedYaw->kP;
		PIDAngleSpeedYawParams[1] = *m_Config->Instance()->pids->pidAngleSpeedYaw->kI;
		PIDAngleSpeedYawParams[2] = *m_Config->Instance()->pids->pidAngleSpeedYaw->kD;
		PIDAngleSpeedYawParams[3] = *m_Config->Instance()->pids->pidAngleSpeedYaw->thOut;
		PIDAngleSpeedYawParams[4] = *m_Config->Instance()->pids->pidAngleSpeedYaw->thIOut;

		PIDAutoAimInputParams[0] = *m_Config->Instance()->pids->pidAutoAimInput->kP;
		PIDAutoAimInputParams[1] = *m_Config->Instance()->pids->pidAutoAimInput->kI;
		PIDAutoAimInputParams[2] = *m_Config->Instance()->pids->pidAutoAimInput->kD;
		PIDAutoAimInputParams[3] = *m_Config->Instance()->pids->pidAutoAimInput->thOut;
		PIDAutoAimInputParams[4] = *m_Config->Instance()->pids->pidAutoAimInput->thIOut;

		//kAngleSpeedFilterCoef = *m_Config->Instance()->mutable_gimbal()->kanglespeedfiltercoef();

		m_GimbalCtrlMode = GimbalCtrlMode::Init;
		m_FlagInitGimbal = true;
		m_FlagInitGyro = true;

		m_PIDAngleEcd[Pitch].SetParams(PIDAngleEcdPitchParams);
		m_PIDAngleEcd[Pitch].SetFlagAngleLoop();

		m_PIDAngleGyro[Pitch].SetParams(PIDAngleGyroPitchParams);
		m_PIDAngleGyro[Pitch].SetFlagAngleLoop();

		m_PIDAngleGyroAutoAim[Pitch].SetParams(PIDAngleGyroPitchAutoAimParams);
		m_PIDAngleGyroAutoAim[Pitch].SetFlagAngleLoop();

		m_PIDAngleSpeed[Pitch].SetParams(PIDAngleSpeedPitchParams);

		m_PIDAutoAimInput[Pitch].SetParams(PIDAutoAimInputParams);
		m_PIDAutoAimInput[Pitch].SetFlagAngleLoop();

		m_PIDAngleEcd[Yaw].SetParams(PIDAngleEcdYawParams);
		m_PIDAngleEcd[Yaw].SetFlagAngleLoop();

		m_PIDAngleGyro[Yaw].SetParams(PIDAngleGyroYawParams);
		m_PIDAngleGyro[Yaw].SetFlagAngleLoop();

		m_PIDAngleGyroAutoAim[Yaw].SetParams(PIDAngleGyroYawAutoAimParams);
		m_PIDAngleGyroAutoAim[Yaw].SetFlagAngleLoop();

		m_PIDAngleSpeed[Yaw].SetParams(PIDAngleSpeedYawParams);

		m_PIDAutoAimInput[Yaw].SetParams(PIDAutoAimInputParams);
		m_PIDAutoAimInput[Yaw].SetFlagAngleLoop();

		FirstOrderFilter ecdAngleFilter(0.25, kGimbalCtrlPeriod);
		m_EcdAngleFilters.fill(ecdAngleFilter);

		FirstOrderFilter rpmFilter(0.15, kGimbalCtrlPeriod);
		m_AngleSpeedFilters.fill(rpmFilter);

		m_LastEcdTimeStamp.fill(std::chrono::high_resolution_clock::time_point());
		m_VoltageSend.fill(0);

		/*m_GyroA204YawListener->AddOnChange([](const GyroA204Status<GyroType::Yaw>& value) {
			SPDLOG_TRACE("@ImuYaw=[$ZAngleYaw={},$ZSpeedYaw={}]",
				value.m_ZAxisAngle, value.m_ZAxisSpeed); });
		m_GyroA204PitchListener->AddOnChange([](const GyroA204Status<GyroType::Pitch>& value) {
			SPDLOG_TRACE("@ImuPitch=[$ZAnglePitch={},$ZSpeedPitch={}]",
				value.m_ZAxisAngle, value.m_ZAxisSpeed); });*/
		/*m_GyroListener->AddOnChange([](const GyroA110Status<GyroType::Gimbal>& value) {
			SPDLOG_TRACE("@GyroGimbalOnChange=[$yaw={},$yawSpeed={},$pitch={},$pitchSpeed={}]",
				value.m_Yaw, value.m_ZAngleSpeed, value.m_Pitch, value.m_YAngleSpeed); });*/

		/*m_GyroListener->AddOnChange([](const GyroA110Status<GyroType::Gimbal>& value) {
			SPDLOG_TRACE("@GimbalImu=[$yaw={},$yawSpeed={}]", value.m_Yaw, value.m_ZAngleSpeed);
		});*/

		//m_VoltageSetFilters[Pitch].SetState(0.09, 0.002);
		//m_VoltageSetFilters[Yaw].SetState(0.09, 0.002);
	}

	//GimbalCtrlMode GimbalCtrlSrc() { return m_GimbalCtrlMode.load(); }

	//double RelativeAngleToChassis()
	//{
	//	return RelativeEcdToRad(m_EcdYaw.load(), kYawEcdMid); //[TODO]负号？
	//}

	void UpdateGimbalSensorFeedback()
	{
		m_MotorsStatus = m_MotorsListener->Get();
		m_GimbalSensorValues.rc = m_RCListener->Get();
		m_GimbalSensorValues.keyboardMode = (m_GimbalSensorValues.rc.sw[0] == kRCSwUp && m_GimbalSensorValues.rc.sw[1] == kRCSwUp);

		m_GimbalSensorValues.relativeAngle[Pitch] = RelativeEcdToRad(m_MotorsStatus.m_Encoding[Pitch], kPitchEcdMid);
		m_GimbalSensorValues.relativeAngle[Yaw] = RelativeEcdToRad(m_MotorsStatus.m_Encoding[Yaw], kYawEcdMid);

		m_GimbalSensorValues.imu = m_GyroListener->Get();
		m_GimbalSensorValues.imu.m_XAxisSpeed *= kDegreeToRadCoef;
		m_GimbalSensorValues.imu.m_Roll *= kDegreeToRadCoef;
		m_GimbalSensorValues.imu.m_YAxisSpeed *= kDegreeToRadCoef;
		m_GimbalSensorValues.imu.m_Pitch *= kDegreeToRadCoef;
		m_GimbalSensorValues.imu.m_ZAxisSpeed *= kDegreeToRadCoef;
		m_GimbalSensorValues.imu.m_Yaw *= kDegreeToRadCoef;
		m_GimbalSensorValues.imu.m_ZAxisSpeed = cos(m_GimbalSensorValues.relativeAngle[Pitch]) * m_GimbalSensorValues.imu.m_ZAxisSpeed
				- sin(m_GimbalSensorValues.relativeAngle[Pitch]) * m_GimbalSensorValues.imu.m_XAxisSpeed;
		SPDLOG_TRACE("@RelativeAngle=[$PitchAngle={},$YawAngle={}]",
			m_GimbalSensorValues.relativeAngle[Pitch],
			m_GimbalSensorValues.relativeAngle[Yaw]);
		//std::cout << m_GimbalSensorValues.relativeAngle[Pitch] << ' ' << m_GimbalSensorValues.relativeAngle[Yaw] << std::endl;
		/*SPDLOG_TRACE("@RotYaw=[$YawAngle={},$YawSpeed={}]",
			m_GimbalSensorValues.imu.m_Yaw,
			m_GimbalSensorValues.imu.m_ZAxisSpeed);*/
		/*if(!m_FlagInitGimbal)
			m_GimbalSensorValues.imuPitch.m_ZAxisAngle = ClampLoop(static_cast<double>(m_GimbalSensorValues.imuPitch.m_ZAxisAngle), m_GyroAngleZeroPoints[Pitch], m_GyroAngleZeroPoints[Pitch] + M_PI * 2);*/

		auto tempAutoAimStatus = m_AutoAimStatusListener->Get();
		if (m_GimbalSensorValues.autoAimStatus.m_Timestamp != tempAutoAimStatus.m_Timestamp)
			m_GimbalSensorValues.autoAimStatus = tempAutoAimStatus;
		else
		{
			m_GimbalSensorValues.autoAimStatus.m_Found = false;
			m_GimbalSensorValues.autoAimStatus.m_Pitch = 0;
			m_GimbalSensorValues.autoAimStatus.m_Yaw = 0;
			m_GimbalSensorValues.autoAimStatus.m_Dist = 0;
		}
		//m_GimbalSensorValues.autoAimStatus = m_AutoAimStatusListener->Get();
		//std::cerr << m_GimbalSensorValues.autoAimStatus.m_Pitch << '\t' << m_GimbalSensorValues.autoAimStatus.m_Yaw << '\t' << m_GimbalSensorValues.autoAimStatus.m_Dist << std::endl;
		/*if(!m_FlagInitGimbal)
			m_GimbalSensorValues.imuYaw.m_ZAxisAngle = ClampLoop(static_cast<double>(m_GimbalSensorValues.imuYaw.m_ZAxisAngle), m_GyroAngleZeroPoints[Yaw], m_GyroAngleZeroPoints[Yaw] + M_PI * 2);*/
		
		//std::swap(m_GimbalSensorValues.imu.m_Roll, m_GimbalSensorValues.imu.m_Pitch);
		//std::swap(m_GimbalSensorValues.imu.m_Wx, m_GimbalSensorValues.imu.m_Wy);
		//m_GimbalSensorValues.imu.m_Pitch = -m_GimbalSensorValues.imu.m_Pitch;
		////gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		//m_GimbalSensorValues.imu.m_Wz = cos(m_GimbalSensorValues.relativeAngle[Pitch]) * m_GimbalSensorValues.imu.m_Wz
		//	- sin(m_GimbalSensorValues.relativeAngle[Pitch]) * m_GimbalSensorValues.imu.m_Wx;

		/*SPDLOG_TRACE("@IMUAngle=[$GPitch={},$GYaw={}]",
			m_GimbalSensorValues.imuPitch.m_ZAxisAngle,
			m_GimbalSensorValues.imuYaw.m_ZAxisAngle);
		SPDLOG_TRACE("@IMUSpeed=[$WPitch={},$WYaw={}]",
			m_GimbalSensorValues.imuPitch.m_ZAxisSpeed,
			m_GimbalSensorValues.imuYaw.m_ZAxisSpeed);*/
		/*SPDLOG_DEBUG("@IMUMagnetometer=[$roll_h={},$pitch_h={},$yaw_h={}]",
			m_GimbalSensorValues.imu.m_Hx,
			m_GimbalSensorValues.imu.m_Hy,
			m_GimbalSensorValues.imu.m_Hz);*/

		/*SPDLOG_TRACE("@MotorEncoder=[$EPitch={},$EYaw={}]",
			m_MotorsStatus.m_Encoding[Pitch],
			m_MotorsStatus.m_Encoding[Yaw]);*/

	}


	//设置云台角度输入来源
	void GimbalCtrlModeSet();

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
			while (6000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();

			/*SPDLOG_TRACE("@Interval=[$t={}]",
				std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count() / 1000.0);*/

			
			//m_EcdYaw = m_MotorsStatus.m_Encoding[Yaw];

			UpdateGimbalSensorFeedback();

			GimbalCtrlModeSet();
			m_Status.m_CtrlMode = m_GimbalCtrlMode;
			m_Status.m_RelativeAngleToChassis = m_GimbalSensorValues.relativeAngle[Yaw];
			m_Status.m_AutoAimShoot = m_GimbalSensorValues.autoAimStatus.m_FlagFire;
			m_GimbalStatusSender->Set(m_Status);
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
	ossian::Utils::ConfigLoader<Config::ConfigSchema>* m_Config;
	Gimbal* m_Gimbal;
	ossian::IOData<RemoteStatus>* m_RCListener;  //遥控器
	ossian::IOData<GyroA110Status<GyroType::Gimbal>>* m_GyroListener;
	ossian::IOData<AutoAimStatus>* m_AutoAimStatusListener;

	std::array<GimbalAngleMode, 2> m_CurGimbalAngleMode;
	GimbalCtrlMode m_GimbalCtrlMode;
	GimbalMotorsModel m_MotorsStatus;
	//std::atomic<uint16_t> m_EcdYaw{};

	struct GimbalSensorFeedback
	{
		double relativeAngle[kNumGimbalMotors];
		RemoteStatus rc;	 //遥控器数据
		GyroA110Status<GyroType::Gimbal> imu;
		AutoAimStatus autoAimStatus;
		bool keyboardMode;
		//double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //云台imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
	} m_GimbalSensorValues;

	bool m_FlagInitGimbal, m_FlagInitGyro; //上电后陀螺仪静置5秒用于自校正
	std::chrono::high_resolution_clock::time_point m_TimestampInit;
	GimbalStatus m_Status;
	//std::atomic<uint16_t> m_EcdYaw;
	ossian::IOData<GimbalStatus>* m_GimbalStatusSender;

	std::array<double, kNumGimbalMotors> m_AngleInput;
	std::array<double, kNumGimbalMotors> m_LastEcdAngle;
	std::array<std::chrono::high_resolution_clock::time_point, kNumGimbalMotors> m_LastEcdTimeStamp;

	std::array<FirstOrderFilter, kNumGimbalMotors> m_EcdAngleFilters;
	std::array<FirstOrderFilter, kNumGimbalMotors> m_AngleSpeedFilters;
	/*double m_YawAdd, m_PitchAdd; //角度增量rad
	double m_LastYaw, m_LastPitch;*/
	std::array<double, kNumGimbalMotors> m_GyroAngleSet; //累加 陀螺仪模式
	std::array<double, kNumGimbalMotors> m_EcdAngleSet; //累加 编码器模式

	//设定角度--->旋转角速度  旋转角速度-->6020控制电压
	std::array<PIDController, kNumGimbalMotors> m_PIDAutoAimInput;
	std::array<PIDController, kNumGimbalMotors> m_PIDAngleEcd, m_PIDAngleGyro, m_PIDAngleGyroAutoAim, m_PIDAngleSpeed;
	std::array<double, kNumGimbalMotors> m_VoltageSend;

	//std::array<FirstOrderFilter, kNumGimbalMotors> m_VoltageSetFilters;
};


#endif // OSSIAN_GIMBAL_HPP