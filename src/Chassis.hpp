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
	//麦轮运动
	static constexpr double WHEEL_RADIUS = 76 / 1000;
	static constexpr double WHEEL_XN = 175 / 1000;
	static constexpr double WHEEL_YN = 232.5 / 1000;
	static constexpr double LIMIT_WHEEL_SPEED = 5;  //单个麦轮的最大速度
	static constexpr double MOTOR_RPM_TO_WHEEL_SPEED = 1 / 9.5;

	//底盘功率控制
	static constexpr double LIMIT_BUFFER_TOTAL_CURRENT = 16000;
	static constexpr double LIMIT_POWER_TOTAL_CURRENT = 20000;
	static constexpr double SPCAP_WARN_VOLTAGE = 1;

	//遥控器解析
	static constexpr size_t CHASSIS_X_CHANNEL = 1; //控制底盘 前后 速度的遥控器通道
	static constexpr size_t CHASSIS_Y_CHANNEL = 0; //控制底盘 左右 速度的遥控器通道
	static constexpr size_t CHASSIS_Z_CHANNEL = 2; //控制底盘 旋转 速度的遥控器通道
	static constexpr size_t CHASSIS_MODE_CHANNEL = 0; //选择底盘状态的开关通道

	static constexpr uint16_t RC_SW_UP = 1;
	static constexpr uint16_t RC_SW_MID = 3;
	static constexpr uint16_t RC_SW_DOWN = 2;

	static constexpr int16_t CHASSIS_RC_DEADBAND = 10; //摇杆死区
	static constexpr double CHASSIS_VX_RC_SEN = 0.006; //遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
	static constexpr double CHASSIS_VY_RC_SEN = 0.005; //遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
	static constexpr double CHASSIS_WZ_RC_SEN = 0.01;  //不跟随云台的时候，遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例

	//底盘运动
	static constexpr double CHASSIS_VX_MAX = 4.5; // m/s
	static constexpr double CHASSIS_VY_MAX = 1.5; // m/s
	static constexpr double TOP_WZ = 3;  //底盘陀螺旋转速度 rad/s

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
		DISABLE,				 //失能
		FOLLOW_GIMBAL_YAW,		 //跟随云台
		FOLLOW_CHASSIS_YAW,		 //遥控器控制底盘旋转，底盘自身角速度闭环
		TOP,					 //小陀螺
		ANGLEOPENLOOP			 //单独调试底盘
	};
	
	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int id)
	{
		m_Motors[position] = 
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

	//通过控制底盘电机的电流来实现
	void ChassisPowerCtrl();

	//将遥控器杆量转为底盘三轴运动期望
	void RCToChassisSpeed();

	//通过遥控器设置底盘运动模式
	void ChassisModeSet();

	//将底盘三轴控制量转换成电流发给电机
	void ChassisCtrl();

	//根据当前模式，计算底盘三轴速度
	void ChassisAxisSpeedSet();

	auto MotorReceiveProc(std::shared_ptr<ossian::DJIMotor> motor, MotorPosition position)->void
	{
		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[0] && m_MotorMsgCheck[1] && m_MotorMsgCheck[2] && m_MotorMsgCheck[3]))  //俯视，左前，左后，右后，右前，逆时针
			return;

		//chassis_task
		UpdateChassisSensorFeedback();

		ChassisModeSet();
		//[TODO] 模式切换过渡

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
		struct RC { int16_t ch[5]; 	char s[2]; } rc;	 //遥控器数据
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	 //底盘imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		double spCapInputVtg, spCapCurVtg, spCapInputCrt, spCapTargetPwr;    //超级电容数据
		double refereeCurPwr, refereeCurBuf, refereeMaxPwr, refereeMaxBuf;   //裁判系统数据
		double relativeAngle; //底盘坐标系与云台坐标系的夹角 当前yaw编码值减去中值 rad
		//double gimbalEcdYaw;
	} m_ChassisSensorValues;

	double m_VxSet, m_VySet, m_WzSet; //三轴速度期望
	double m_AngleSet;  //底盘角度目标值
	

	ChassisMode m_CurChassisMode, m_LastChassisMode;
	std::array<bool, 4> m_MotorMsgCheck;
	Eigen::Vector4d m_WheelSpeedSet;
	Eigen::Matrix<double, 4, 3> m_WheelKinematicMat;
	std::array<double, 4> m_CurrentSend;

	PIDController m_PIDChassisAngle; //底盘要旋转的角度--->底盘旋转角速度
	std::array<PIDController, 4> m_PIDChassisSpeed; //麦轮转速--->3508电流
};

#endif // OSSIAN_CHASSIS_HPP