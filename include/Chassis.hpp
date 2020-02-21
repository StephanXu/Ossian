#ifndef OSSIAN_CHASSIS_HPP
#define OSSIAN_CHASSIS_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"

#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <cmath>
#include <array>


class Chassis
{	
public:
	//�����˶�
	static constexpr double WHEEL_RADIUS = 76 / 1000;
	static constexpr double WHEEL_XN = 175 / 1000;
	static constexpr double WHEEL_YN = 232.5 / 1000;
	static constexpr double LIMIT_WHEEL_SPEED = 5;  //�������ֵ�����ٶ�
	static constexpr double MOTOR_RPM_TO_WHEEL_SPEED = 1 / 9.5;

	//���̹��ʿ���
	static constexpr double LIMIT_BUFFER_TOTAL_CURRENT = 16000;
	static constexpr double LIMIT_POWER_TOTAL_CURRENT = 20000;
	static constexpr double SPCAP_WARN_VOLTAGE = 1;

	//ң��������
	static constexpr size_t CHASSIS_X_CHANNEL = 1; //���Ƶ��� ǰ�� �ٶȵ�ң����ͨ��
	static constexpr size_t CHASSIS_Y_CHANNEL = 0; //���Ƶ��� ���� �ٶȵ�ң����ͨ��
	static constexpr size_t CHASSIS_Z_CHANNEL = 2; //���Ƶ��� ��ת �ٶȵ�ң����ͨ��
	static constexpr size_t CHASSIS_MODE_CHANNEL = 0; //ѡ�����״̬�Ŀ���ͨ��

	static constexpr uint16_t RC_SW_UP = 1;
	static constexpr uint16_t RC_SW_MID = 3;
	static constexpr uint16_t RC_SW_DOWN = 2;

	static constexpr int16_t CHASSIS_RC_DEADBAND = 10; //ҡ������
	static constexpr double CHASSIS_VX_RC_SEN = 0.006; //ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
	static constexpr double CHASSIS_VY_RC_SEN = 0.005; //ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
	static constexpr double CHASSIS_WZ_RC_SEN = 0.01;  //��������̨��ʱ��ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���

	//�����˶�
	static constexpr double CHASSIS_VX_MAX = 4.5; // m/s
	static constexpr double CHASSIS_VY_MAX = 1.5; // m/s
	static constexpr double TOP_WZ = 3;  //����������ת�ٶ� rad/s

	OSSIAN_SERVICE_SETUP(Chassis(ossian::MotorManager* motorManager))
		: m_MotorManager(motorManager)
	{
		double coef = WHEEL_XN + WHEEL_YN;
		m_WheelKinematicMat << 1, -1, -coef,
							   1,  1, -coef,
							   1, -1,  coef,
							   1,  1,  coef;
		m_AngleSet = 0;
		m_MotorMsgCheck.fill(false);

		PIDController pidWheelSpeed(15000, 10, 0);
		pidWheelSpeed.SetThresOutput(16000);  //max 16384
		pidWheelSpeed.SetThresIntegral(2000);
		m_PIDChassisSpeed.fill(pidWheelSpeed);

		m_PIDChassisAngle.SetPIDParams(40, 0, 0);
		m_PIDChassisAngle.SetThresOutput(6);
		m_PIDChassisAngle.SetThresIntegral(0.2);
	}

	enum MotorPosition
	{
		LF,LR,RR,RF
	};

	enum ChassisMode
	{
		DISABLE,             //ʧ��
		FOLLOW_GIMBAL_YAW,   //������̨
		FOLLOW_CHASSIS_YAW,  //ң�������Ƶ�����ת������������ٶȱջ�
		TOP					 //С����
	};
	
	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int id)
	{
		if(position == LF || position == LR || position == RR || position == RF)
			m_MotorManager->AddMotor<ossian::DJIMotor>(
				location,
				id,
				[this,position](std::shared_ptr<ossian::DJIMotor> motor)
				{
					MotorReceiveProc(motor, position);
				});
	}

	void UpdateChassisSensorFeedback() { /*m_RelativeAngle = yaw_ecd - mid_ecd;*/ }

	void CalcWheelSpeed();

	//ͨ�����Ƶ��̵���ĵ�����ʵ��
	void ChassisPowerCtrl();

	//��ң��������תΪ���������˶�����
	void RCToChassisSpeed();

	//ͨ��ң�������õ����˶�ģʽ
	void ChassisModeSet();

	//���������������ת���ɵ����������
	void ChassisCtrl();

	//���ݵ�ǰģʽ��������������ٶ�
	void ChassisAxisSpeedSet();

	auto MotorReceiveProc(std::shared_ptr<ossian::DJIMotor> motor, MotorPosition position)->void
	{
		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[0] && m_MotorMsgCheck[1] && m_MotorMsgCheck[2] && m_MotorMsgCheck[3]))  //���ӣ���ǰ������Һ���ǰ����ʱ��
			return;

		//chassis_task
		UpdateChassisSensorFeedback();

		ChassisModeSet();
		//[TODO] ģʽ�л�����

		ChassisAxisSpeedSet();
		
		ChassisCtrl();
		m_MotorMsgCheck.fill(false);
	}

private:
	ossian::MotorManager* m_MotorManager; 	
	std::array<std::shared_ptr<ossian::DJIMotor>, 4> m_Motors; 	
	std::chrono::high_resolution_clock::time_point m_LastRefresh;

	struct ChassisSensorFeedback 
	{ 
		struct RC { int16_t ch[5]; 	char s[2]; } rc;	 //ң��������
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //����imu���� [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		double spCapInputVtg, spCapCurVtg, spCapInputCrt, spCapTargetPwr;    //������������
		double refereeCurPwr, refereeCurBuf, refereeMaxPwr, refereeMaxBuf;   //����ϵͳ����
		double relativeAngle; //��������ϵ����̨����ϵ�ļн� ��ǰyaw����ֵ��ȥ��ֵ rad
		//double gimbalEcdYaw;
	} m_ChassisSensorValues;

	double m_VxSet, m_VySet, m_WzSet; //�����ٶ�����
	double m_AngleSet;  //���̽Ƕ�Ŀ��ֵ
	

	ChassisMode m_CurChassisMode, m_LastChassisMode;
	std::array<bool, 4> m_MotorMsgCheck;
	Eigen::Vector4d m_WheelSpeedSet;
	Eigen::Matrix<double, 4, 3> m_WheelKinematicMat;
	std::array<double, 4> m_CurrentSend;

	PIDController m_PIDChassisAngle; //����Ҫ��ת�ĽǶ�--->������ת���ٶ�
	std::array<PIDController, 4> m_PIDChassisSpeed; //����ת��--->3508����
};

#endif // OSSIAN_CHASSIS_HPP