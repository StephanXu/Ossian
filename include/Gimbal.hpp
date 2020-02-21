#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"

#include <chrono>
#include <memory>


class Gimbal
{
	//��̨����λ��
	static constexpr uint16_t PITCH_MID_ECD = 100;
	static constexpr uint16_t YAW_MID_ECD = 200;
	static constexpr uint16_t PITCH_MAX_ECD = 300;
	static constexpr uint16_t YAW_MAX_ECD = 600;
	static constexpr uint16_t PITCH_MIN_ECD = 50;
	static constexpr uint16_t YAW_MIN_ECD = 10;
	static constexpr double   PITCH_MID_RAD = PITCH_MID_ECD * MOTOR_ECD_TO_RAD_COEF;
	static constexpr double   YAW_MID_RAD = YAW_MID_ECD * MOTOR_ECD_TO_RAD_COEF;

	//�����С�� ��ԣ���ֵ�ģ��Ƕ�
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

	//ң��������
	static constexpr int16_t GIMBAL_RC_DEADBAND = 10; //ҡ������
	static constexpr size_t YAW_CHANNEL = 4;
	static constexpr size_t PITCH_CHANNEL = 3;
	static constexpr size_t GIMBAL_MODE_CHANNEL = 1; //ѡ����̨״̬�Ŀ���ͨ��
	static constexpr uint16_t RC_SW_UP = 1;
	static constexpr uint16_t RC_SW_MID = 3;
	static constexpr uint16_t RC_SW_DOWN = 2;

	static constexpr double YAW_RC_SEN = -0.000005;
	static constexpr double PITCH_RC_SEN = -0.000006; //0.005

public:
	OSSIAN_SERVICE_SETUP(Gimbal(ossian::MotorManager* motorManager))
		: m_MotorManager(motorManager)
	{
		m_CtrlMode = RC;
		m_LastEcdTimeStampPitch = m_LastEcdTimeStampYaw = std::chrono::high_resolution_clock::time_point();

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

		UpdateGimbalSensorFeedback();
		m_GyroPitchAngleSet = m_GimbalSensorValues.gyroY;
		m_GyroYawAngleSet = m_GimbalSensorValues.gyroZ;
		m_EcdPitchAngleSet = PITCH_MID_RAD;
		m_EcdYawAngleSet = YAW_MID_RAD;
	}

	enum MotorPosition
	{
		Pitch, Yaw
	};
	
	enum GimbalAngleMode
	{
		GYROANGLE, ECDANGLE
	};

	enum GimbalCtrlMode
	{
		RC, MOUSE, AUTOAIM, DISABLE
	};

	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int id)
	{
		if(position == Pitch)
			m_MotorManager->AddMotor<ossian::DJIMotor>(
				location,
				id,
				[this](std::shared_ptr<ossian::DJIMotor> motor)
				{
					MotorPitchReceiveProc(motor);
				});
		else if(position == Yaw)
			m_MotorManager->AddMotor<ossian::DJIMotor>(
				location,
				id,
				[this](std::shared_ptr<ossian::DJIMotor> motor)
				{
					MotorYawReceiveProc(motor);
				});
	}

	void UpdateGimbalSensorFeedback() {}

	//��ò����������ĽǶ�
	double PitchCtrlInputProc();
	double YawCtrlInputProc();

	//����ң�����ݣ����ýǶ�����ֵrad
	void SetPitch(double angleInput, uint16_t curEcd);
	void SetYaw(double angleInput, uint16_t curEcd);

	//˫��pid���� 
	void CtrlPitch();
	void CtrlYaw();

	auto MotorPitchReceiveProc(std::shared_ptr<ossian::DJIMotor> motor)->void
	{
		UpdateGimbalSensorFeedback();
		SetPitch(PitchCtrlInputProc(), motor->Status().m_Encoding);
		CtrlPitch();
	}

	auto MotorYawReceiveProc(std::shared_ptr<ossian::DJIMotor> motor)->void
	{
		UpdateGimbalSensorFeedback();
		SetYaw(YawCtrlInputProc(), motor->Status().m_Encoding);
		CtrlYaw();
	}

private:
	ossian::MotorManager* m_MotorManager;  	
	std::array<std::shared_ptr<ossian::DJIMotor>, 2> m_Motors;  	
	std::chrono::high_resolution_clock::time_point m_LastRefresh;

	GimbalAngleMode m_CurGimbalMode, m_LastGimbalMode;
	GimbalCtrlMode m_CtrlMode;
	struct GimbalSensorFeedback
	{
		struct RC { int16_t ch[5]; 	char s[2]; } rc;	 //ң��������
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //����imu���� [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
	} m_GimbalSensorValues;

	double m_LastEcdAnglePitch, m_LastEcdAngleYaw;
	std::chrono::high_resolution_clock::time_point m_LastEcdTimeStampPitch, m_LastEcdTimeStampYaw;

	/*double m_YawAdd, m_PitchAdd; //�Ƕ�����rad
	double m_LastYaw, m_LastPitch;*/
	double m_GyroPitchAngleSet, m_GyroYawAngleSet; //�ۼ� ������ģʽ
	double m_EcdPitchAngleSet, m_EcdYawAngleSet; //�ۼ� ������ģʽ

	PIDController m_PIDPitchAngleEcd, m_PIDPitchAngleGyro, m_PIDPitchAngleSpeed; //�趨�Ƕ�--->��ת���ٶ�  ��ת���ٶ�-->6020���Ƶ�ѹ
	PIDController m_PIDYawAngleEcd, m_PIDYawAngleGyro, m_PIDYawAngleSpeed;
};

#endif // OSSIAN_GIMBAL_HPP