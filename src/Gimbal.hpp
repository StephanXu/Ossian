#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"
#include "Remote.hpp"

#include <chrono>
#include <memory>
#include <atomic>

class Gimbal
{
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
	const double PITCH_MAX_RELATIVE_ANGLE = RelativeEcdToRad(PITCH_MAX_ECD, PITCH_MID_ECD);
	const double PITCH_MIN_RELATIVE_ANGLE = RelativeEcdToRad(PITCH_MIN_ECD, PITCH_MID_ECD);
	const double YAW_MAX_RELATIVE_ANGLE = RelativeEcdToRad(YAW_MAX_ECD, YAW_MID_ECD);
	const double YAW_MIN_RELATIVE_ANGLE = RelativeEcdToRad(YAW_MIN_ECD, YAW_MID_ECD);
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
		GYROANGLE, ECDANGLE
	};

	enum GimbalInputSrc
	{
		RC, MOUSE, AUTOAIM, DISABLE
	};

public:
	enum MotorPosition
	{
		Pitch, Yaw
	};
	
	OSSIAN_SERVICE_SETUP(Gimbal(ossian::MotorManager* motorManager, IRemote* remote))
		: m_MotorManager(motorManager), m_RC(remote)
	{
		m_GimbalCtrlSrc = RC;
		m_FlagInitPitch = m_FlagInitYaw = true;

		m_PIDPitchAngleEcd.SetPIDParams(15,0,0);
		m_PIDPitchAngleEcd.SetThresOutput(10);

		m_PIDPitchAngleGyro.SetPIDParams(15, 0, 0);
		m_PIDPitchAngleGyro.SetThresOutput(10);

		m_PIDPitchAngleSpeed.SetPIDParams(2900, 60, 0);
		m_PIDPitchAngleSpeed.SetThresIntegral(10000);
		m_PIDPitchAngleSpeed.SetThresOutput(30000);

		m_PIDYawAngleEcd.SetPIDParams(8, 0, 0);
		m_PIDYawAngleEcd.SetThresOutput(10);

		m_PIDYawAngleGyro.SetPIDParams(26, 0, 0.3);
		m_PIDYawAngleGyro.SetThresOutput(10);

		m_PIDYawAngleSpeed.SetPIDParams(3600, 20, 0);
		m_PIDYawAngleSpeed.SetThresIntegral(5000);
		m_PIDYawAngleSpeed.SetThresOutput(30000);

		
	}

	void InitPitch()
	{
		m_CurGimbalAngleMode = ECDANGLE; //or gyro
		m_LastEcdTimeStampPitch = std::chrono::high_resolution_clock::time_point();

		m_GyroPitchAngleSet = m_GimbalSensorValues.gyroY;
		m_EcdPitchAngleSet = PITCH_MID_RAD;
		
		m_PIDPitchAngleEcd.Reset();
		m_PIDPitchAngleGyro.Reset();
		m_PIDPitchAngleSpeed.Reset();
		
		m_FlagInitPitch = false;
	}

	void InitYaw()
	{
		m_CurGimbalAngleMode = ECDANGLE; //or gyro
		m_LastEcdTimeStampYaw = std::chrono::high_resolution_clock::time_point();

		m_GyroYawAngleSet = m_GimbalSensorValues.gyroZ;
		m_EcdYawAngleSet = YAW_MID_RAD;

		m_PIDYawAngleEcd.Reset();
		m_PIDYawAngleGyro.Reset();
		m_PIDYawAngleSpeed.Reset();

		m_FlagInitYaw = false;
	}

	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int id,
				  const unsigned int writerCanId)
	{
		if (position == Pitch)
		{
			m_Motors[position] =
				m_MotorManager->AddMotor<ossian::DJIMotor>(
					location,
					m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
					[this](const std::shared_ptr<ossian::DJIMotor>& motor)
					{
						MotorPitchReceiveProc(motor);
					},
					id);
		}
		else if (position == Yaw)
		{
			m_Motors[position] =
				m_MotorManager->AddMotor<ossian::DJIMotor>(
					location,
					m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
					[this](const std::shared_ptr<ossian::DJIMotor>& motor)
					{
						MotorYawReceiveProc(motor);
					},
					id);
		}
	}

	double RelativeAngleToChassis() { return RelativeEcdToRad(m_YawEcd.load(), YAW_MID_ECD); } //[TODO]负号？

	void UpdateGimbalSensorFeedback()
	{
		m_GimbalSensorValues.rc = m_RC->Status();
	}
	//设置云台角度输入来源
	void GimbalCtrlSrcSet();

	//获得操作手期望的角度
	double PitchCtrlInputProc();
	double YawCtrlInputProc();

	//根据遥控数据，设置角度期望值rad
	void SetPitch(double angleInput, uint16_t curEcd);
	void SetYaw(double angleInput, uint16_t curEcd);

	//双环pid计算 
	void CtrlPitch();
	void CtrlYaw();

	auto MotorPitchReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor)->void
	{
		UpdateGimbalSensorFeedback();
		if (m_FlagInitPitch)
			InitPitch();
		GimbalCtrlSrcSet();
		//[TODO] 模式切换过渡
		SetPitch(PitchCtrlInputProc(), motor->Status().m_Encoding);
		CtrlPitch();
	}

	auto MotorYawReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor)->void
	{
		UpdateGimbalSensorFeedback();
		if (m_FlagInitYaw)
			InitYaw();
		GimbalCtrlSrcSet();
		//[TODO] 模式切换过渡
		m_YawEcd = motor->Status().m_Encoding;
		SetYaw(YawCtrlInputProc(), motor->Status().m_Encoding);
		CtrlYaw();
	}

private:
	ossian::MotorManager* m_MotorManager;  	
	std::array<std::shared_ptr<ossian::DJIMotor>, 2> m_Motors;  	
	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	IRemote* m_RC;  //遥控器

	GimbalAngleMode m_CurGimbalAngleMode, m_LastGimbalAngleMode;
	GimbalInputSrc m_GimbalCtrlSrc;
	struct GimbalSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //云台imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
	} m_GimbalSensorValues;
	std::atomic<uint16_t> m_YawEcd;

	bool m_FlagInitPitch, m_FlagInitYaw;

	double m_LastEcdAnglePitch, m_LastEcdAngleYaw;
	std::chrono::high_resolution_clock::time_point m_LastEcdTimeStampPitch, m_LastEcdTimeStampYaw;

	/*double m_YawAdd, m_PitchAdd; //角度增量rad
	double m_LastYaw, m_LastPitch;*/
	double m_GyroPitchAngleSet, m_GyroYawAngleSet; //累加 陀螺仪模式
	double m_EcdPitchAngleSet, m_EcdYawAngleSet; //累加 编码器模式

	PIDController m_PIDPitchAngleEcd, m_PIDPitchAngleGyro, m_PIDPitchAngleSpeed; //设定角度--->旋转角速度  旋转角速度-->6020控制电压
	PIDController m_PIDYawAngleEcd, m_PIDYawAngleGyro, m_PIDYawAngleSpeed;
};

#endif // OSSIAN_GIMBAL_HPP