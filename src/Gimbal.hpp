#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"
#include "Remote.hpp"

#include <chrono>
#include <array>
#include <memory>
#include <atomic>

class Gimbal
{
public:
	static constexpr double MOTOR_ECD_TO_RAD_COEF = 2 * M_PI / 8192;
	//云台特殊位置
	static constexpr uint16_t PITCH_MID_ECD = 100;
	static constexpr uint16_t YAW_MID_ECD = 200;
	static constexpr uint16_t PITCH_MAX_ECD = 300;
	static constexpr uint16_t YAW_MAX_ECD = 600;
	static constexpr uint16_t PITCH_MIN_ECD = 50;
	static constexpr uint16_t YAW_MIN_ECD = 10;
	static constexpr double   PITCH_MID_RAD = PITCH_MID_ECD * MOTOR_ECD_TO_RAD_COEF;
	static constexpr double   YAW_MID_RAD = YAW_MID_ECD * MOTOR_ECD_TO_RAD_COEF;

	//最大最小的 相对（中值的）角度
	const std::array<double, 2> MAX_RELATIVE_ANGLE = { RelativeEcdToRad(PITCH_MAX_ECD, PITCH_MID_ECD),RelativeEcdToRad(YAW_MAX_ECD, YAW_MID_ECD) };
	const std::array<double, 2> MIN_RELATIVE_ANGLE = { RelativeEcdToRad(PITCH_MIN_ECD, PITCH_MID_ECD),RelativeEcdToRad(YAW_MIN_ECD, YAW_MID_ECD) };
	/*
	static constexpr double   PITCH_MAX_RAD = PITCH_MAX_ECD * MOTOR_ECD_TO_RAD_COEF;
	static constexpr double   PITCH_MIN_RAD = PITCH_MIN_ECD * MOTOR_ECD_TO_RAD_COEF;
	static constexpr double   YAW_MAX_RAD   = YAW_MAX_ECD   * MOTOR_ECD_TO_RAD_COEF;
	static constexpr double   YAW_MIN_RAD   = YAW_MIN_ECD   * MOTOR_ECD_TO_RAD_COEF;
	*/

	//遥控器解析
	static constexpr int16_t GIMBAL_RC_DEADBAND = 10; //摇杆死区
	static constexpr size_t YAW_CHANNEL = 4;
	static constexpr size_t PITCH_CHANNEL = 3;
	static constexpr size_t GIMBAL_MODE_CHANNEL = 1; //选择云台状态的开关通道
	static constexpr uint8_t RC_SW_UP = 1;
	static constexpr uint8_t RC_SW_MID = 3;
	static constexpr uint8_t RC_SW_DOWN = 2;

	static constexpr double YAW_RC_SEN = -0.000005;
	static constexpr double PITCH_RC_SEN = -0.000006; //0.005

	enum GimbalAngleMode
	{
		Gyro, Encoding
	};

	enum GimbalInputSrc
	{
		Disable, RC, Mouse, Aimbot, Windmill
	};

	enum MotorPosition
	{
		Pitch, Yaw
	};
	
	OSSIAN_SERVICE_SETUP(Gimbal(ossian::MotorManager* motorManager, IRemote* remote))
		: m_MotorManager(motorManager), m_RC(remote)
	{
		m_GimbalCtrlSrc = RC;
		m_FlagInitGimbal = true;

		m_PIDAngleEcd[Pitch].SetPIDParams(15,0,0);
		m_PIDAngleEcd[Pitch].SetThresOutput(10);

		m_PIDAngleGyro[Pitch].SetPIDParams(15, 0, 0);
		m_PIDAngleGyro[Pitch].SetThresOutput(10);

		m_PIDAngleSpeed[Pitch].SetPIDParams(2900, 60, 0);
		m_PIDAngleSpeed[Pitch].SetThresIntegral(10000);
		m_PIDAngleSpeed[Pitch].SetThresOutput(30000);

		m_PIDAngleEcd[Yaw].SetPIDParams(8, 0, 0);
		m_PIDAngleEcd[Yaw].SetThresOutput(10);

		m_PIDAngleGyro[Yaw].SetPIDParams(26, 0, 0.3);
		m_PIDAngleGyro[Yaw].SetThresOutput(10);

		m_PIDAngleSpeed[Yaw].SetPIDParams(3600, 20, 0);
		m_PIDAngleSpeed[Yaw].SetThresIntegral(5000);
		m_PIDAngleSpeed[Yaw].SetThresOutput(30000);

		
	}

	void InitGimbal()
	{
		m_CurGimbalAngleMode = Encoding; //or gyro
		m_LastEcdTimeStamp.fill(std::chrono::high_resolution_clock::time_point());
		m_MotorMsgCheck.fill(false);
		m_AngleInput.fill(0);

		m_GyroAngleSet[Pitch] = m_GimbalSensorValues.gyroY;
		m_GyroAngleSet[Yaw] = m_GimbalSensorValues.gyroZ;

		m_EcdAngleSet[Pitch] = PITCH_MID_RAD;
		m_EcdAngleSet[Yaw] = YAW_MID_RAD;

		m_PIDAngleEcd[Pitch].Reset();
		m_PIDAngleGyro[Pitch].Reset();
		m_PIDAngleSpeed[Pitch].Reset();
		
		m_PIDAngleEcd[Yaw].Reset();
		m_PIDAngleGyro[Yaw].Reset();
		m_PIDAngleSpeed[Yaw].Reset();

		m_FlagInitGimbal = false;
	}

	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int motorId,
				  const unsigned int writerCanId)->void
	{
		m_Motors[position] =
			m_MotorManager->AddMotor<ossian::DJIMotor>(
				location,
				m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
				[this, position](const std::shared_ptr<ossian::DJIMotor>& motor)
				{
					MotorReceiveProc(motor, position);
				},
				motorId);
	}

	double RelativeAngleToChassis() { return RelativeEcdToRad(m_YawEcd.load(), YAW_MID_ECD); } //[TODO]负号？

	GimbalInputSrc GimbalCtrlSrc() { return m_GimbalCtrlSrc.load(); }

	void UpdateGimbalSensorFeedback()
	{
		m_GimbalSensorValues.rc = m_RC->Status();
	}
	//设置云台角度输入来源
	void GimbalCtrlSrcSet();

	//获得操作手期望的角度输入
	void GimbalCtrlInputProc();

	//根据遥控数据，设置角度期望值rad
	void GimbalExpAngleSet(MotorPosition position);

	//双环pid计算 
	void GimbalCtrlCalc(MotorPosition position);

	auto MotorReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor, MotorPosition position)->void
	{
		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[Pitch] && m_MotorMsgCheck[Yaw]))
			return;

		UpdateGimbalSensorFeedback();
		if (m_FlagInitGimbal)
			InitGimbal();
		GimbalCtrlSrcSet();
		GimbalCtrlInputProc();
		//[TODO] 模式切换过渡

		GimbalExpAngleSet(Pitch);
		GimbalExpAngleSet(Yaw);

		GimbalCtrlCalc(Pitch);
		GimbalCtrlCalc(Yaw);

		std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [this](double x, size_t ix = 0) {m_Motors[ix++]->SetVoltage(x); });
		m_Motors[Pitch]->Writer()->PackAndSend();

		m_MotorMsgCheck.fill(false);
	}


private:
	ossian::MotorManager* m_MotorManager;  	
	std::array<std::shared_ptr<ossian::DJIMotor>, 2> m_Motors;  	
	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	IRemote* m_RC;  //遥控器

	GimbalAngleMode m_CurGimbalAngleMode, m_LastGimbalAngleMode;
	std::atomic<GimbalInputSrc> m_GimbalCtrlSrc;
	std::array<bool, 2> m_MotorMsgCheck;
	struct GimbalSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //云台imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
	} m_GimbalSensorValues;
	std::atomic<uint16_t> m_YawEcd;

	bool m_FlagInitGimbal;

	std::array<double, 2> m_AngleInput;
	std::array<double, 2> m_LastEcdAngle;
	std::array<std::chrono::high_resolution_clock::time_point, 2> m_LastEcdTimeStamp;

	/*double m_YawAdd, m_PitchAdd; //角度增量rad
	double m_LastYaw, m_LastPitch;*/
	std::array<double, 2> m_GyroAngleSet; //累加 陀螺仪模式
	std::array<double, 2> m_EcdAngleSet; //累加 编码器模式

	std::array<PIDController,2> m_PIDAngleEcd, m_PIDAngleGyro, m_PIDAngleSpeed; //设定角度--->旋转角速度  旋转角速度-->6020控制电压
	std::array<double, 4> m_CurrentSend;
};

#endif // OSSIAN_GIMBAL_HPP