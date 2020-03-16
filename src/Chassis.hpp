#ifndef OSSIAN_CHASSIS_HPP
#define OSSIAN_CHASSIS_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gimbal.hpp"
#include "Capacitor.hpp"

#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <cmath>
#include <array>


class Chassis
{
public:
	//麦轮运动
	static constexpr double kWheelRadius = 76 / 1000; ///< m
	static constexpr double kWheelXn = 175 / 1000;    ///< m
	static constexpr double kWheelYn = 232.5 / 1000;  ///< m
	static constexpr double kWheelSpeedLimit = 5;    ///< 单个麦轮的最大速度
	static constexpr double kWheelSpeedToMotorRPMCoef = 11.875;

	//底盘功率控制
	static constexpr double kBufferTotalCurrentLimit = 16000;
	static constexpr double kPowerTotalCurrentLimit = 20000;
	static constexpr double kSpCapWarnVoltage = 12;

	//遥控器解析
	static constexpr size_t kChassisXChannel = 1;    ///< 控制底盘 前后 速度的遥控器通道
	static constexpr size_t kChassisYChannel = 0;    ///< 控制底盘 左右 速度的遥控器通道
	static constexpr size_t kChassisZChannel = 2;    ///< 控制底盘 旋转 速度的遥控器通道 仅当使用Openloop_Z模式时可用
	static constexpr size_t kChassisModeChannel = 0; ///< 选择底盘状态的开关通道

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	static constexpr int16_t kChassisRCDeadband = 10; ///< 摇杆死区
	static constexpr double kChassisVxRCSen = 0.006; ///< 遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
	static constexpr double kChassisVyRCSen = 0.005; ///< 遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
	static constexpr double kChassisWzRCSen = 0.01;  ///< 不跟随云台的时候，遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例

	//底盘运动
	static constexpr double kChassisVxLimit = 4.5; ///< m/s
	static constexpr double kChassisVyLimit = 1.5; ///< m/s
	static double kTopWz;  ///< 底盘陀螺旋转速度 rad/s

	//pid参数 [TODO]底盘旋转角速度闭环
	static std::array<double, 5> PIDWheelSpeedParams;
	static std::array<double, 5> PIDChassisAngleParams;

	enum MotorPosition
	{
		LF, LR, RR, RF
	};
	
	enum ChassisMode
	{
		Disable,				 ///< 失能
		Follow_Gimbal_Yaw,		 ///< 跟随云台
		//Follow_Chassis_Yaw,		 ///< 遥控器控制底盘旋转，底盘角速度闭环。工程采用。
		Top,					 ///< 小陀螺
		Openloop_Z				 ///< 单独调试底盘
	};

	OSSIAN_SERVICE_SETUP(Chassis(ossian::MotorManager* motorManager,
								 IRemote* remote,
								 ICapacitor* capacitor,
								 Gimbal* gimbal,
								 Utils::ConfigLoader* config))
		: m_MotorManager(motorManager)
		, m_RC(remote)
		, m_SpCap(capacitor)
		, m_Gimbal(gimbal)
		, m_Config(config)
	{
		using OssianConfig::Configuration;
		PIDWheelSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidwheelspeed()->kp();
		PIDWheelSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidwheelspeed()->ki();
		PIDWheelSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidwheelspeed()->kd();
		PIDWheelSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidwheelspeed()->thout();
		PIDWheelSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidwheelspeed()->thiout();

		PIDChassisAngleParams[0] = m_Config->Instance<Configuration>()->mutable_pidchassisangle()->kp();
		PIDChassisAngleParams[1] = m_Config->Instance<Configuration>()->mutable_pidchassisangle()->ki();
		PIDChassisAngleParams[2] = m_Config->Instance<Configuration>()->mutable_pidchassisangle()->kd();
		PIDChassisAngleParams[3] = m_Config->Instance<Configuration>()->mutable_pidchassisangle()->thout();
		PIDChassisAngleParams[4] = m_Config->Instance<Configuration>()-> mutable_pidchassisangle()->thiout();

		kTopWz = m_Config->Instance<Configuration>()->mutable_chassis()->ktopwz();

		double coef = kWheelXn + kWheelYn;
		m_WheelKinematicMat << 1, -1, -coef,
			1, 1, -coef,
			1, -1, coef,
			1, 1, coef;

		m_FlagInitChassis = true;

		m_FOFilterVX.SetCoef(0.17);
		m_FOFilterVY.SetCoef(0.33);

		PIDController pidWheelSpeed;
		pidWheelSpeed.SetParams(PIDWheelSpeedParams);
		m_PIDChassisSpeed.fill(pidWheelSpeed);

		m_PIDChassisAngle.SetParams(PIDChassisAngleParams);
	}

	void InitChassis()
	{
		m_AngleSet = 0;
		m_MotorMsgCheck.fill(false);

		m_FOFilterVX.Reset();
		m_FOFilterVY.Reset();

		std::for_each(m_PIDChassisSpeed.begin(), m_PIDChassisSpeed.end(), [](PIDController& x) {x.Reset(); });
		m_PIDChassisAngle.Reset();

		m_FlagInitChassis = false;
	}

	auto AddMotor(const MotorPosition position,
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

	void UpdateChassisSensorFeedback()
	{
		m_ChassisSensorValues.rc = m_RC->Status();
		m_ChassisSensorValues.spCap = m_SpCap->Status();
		m_ChassisSensorValues.relativeAngle = m_Gimbal->RelativeAngleToChassis();
	}

	void CalcWheelSpeedTarget();

	//功率控制：通过减小底盘电机的电流来实现
	void ChassisPowerCtrlByCurrent();

	//将遥控器杆量转为底盘三轴运动期望
	void RCToChassisSpeed();

	//通过遥控器设置底盘运动模式
	void ChassisModeSet();

	//将底盘三轴控制量转换成电流发给电机
	void ChassisCtrl();

	//根据当前模式，计算底盘三轴速度
	void ChassisExpAxisSpeedSet();

	auto MotorReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor,
						  MotorPosition position)->void
	{
		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[LF] && m_MotorMsgCheck[LR] && m_MotorMsgCheck[RR] && m_MotorMsgCheck[RF]))  //俯视，左前，左后，右后，右前，逆时针
			return;

		//chassis_task
		UpdateChassisSensorFeedback();

		if (m_FlagInitChassis)
			InitChassis();

		ChassisModeSet();
		//[TODO] 模式切换过渡

		ChassisExpAxisSpeedSet();

		ChassisCtrl();
		m_SpCap->SetPower(m_ChassisSensorValues.refereeMaxPwr);
		m_MotorMsgCheck.fill(false);
	}

private:
	ossian::MotorManager* m_MotorManager;
	std::array<std::shared_ptr<ossian::DJIMotor>, 4> m_Motors;
	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	Utils::ConfigLoader* m_Config;
	IRemote* m_RC;  //遥控器
	ICapacitor* m_SpCap;
	Gimbal* m_Gimbal;

	bool m_FlagInitChassis;
	struct ChassisSensorFeedback
	{
		RemoteStatus rc;
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ; 	///< 底盘imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		CapacitorStatus spCap;												///< 超级电容数据
		double refereeCurPwr, refereeCurBuf, refereeMaxPwr, refereeMaxBuf;  ///< 裁判系统数据
		double relativeAngle;												///< 底盘坐标系与云台坐标系的夹角 当前yaw编码值减去中值 rad
	} m_ChassisSensorValues;

	double m_VxSet, m_VySet, m_WzSet; //三轴速度期望
	double m_AngleSet;  //底盘角度目标值


	ChassisMode m_CurChassisMode;
	std::array<bool, 4> m_MotorMsgCheck;
	Eigen::Vector4d m_WheelSpeedSet;
	static Eigen::Matrix<double, 4, 3> m_WheelKinematicMat;
	std::array<double, 4> m_CurrentSend;

	FirstOrderFilter m_FOFilterVX, m_FOFilterVY;
	PIDController m_PIDChassisAngle; ///< 底盘要旋转的角度--->底盘旋转角速度  底盘跟随角度环
	std::array<PIDController, 4> m_PIDChassisSpeed; ///< 麦轮转速--->3508电流
};

#endif // OSSIAN_CHASSIS_HPP