#ifndef OSSIAN_CHASSIS_HPP
#define OSSIAN_CHASSIS_HPP

#include <ossian/motors/Motor.hpp>
#include <ossian/motors/DJIMotor.hpp>
#include <ossian/IOData.hpp>
#include <ossian/Pipeline.hpp>
#include <spdlog/spdlog.h>

#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gimbal.hpp"
#include "Capacitor.hpp"
#include "Referee.hpp"

#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <cmath>
#include <array>
#include <thread>


//底盘电机数量
constexpr size_t kNumChassisMotors = 4;

OSSIAN_MULTIPLE_MOTORS_STATUS(ChassisMotorsModel, kNumChassisMotors);

class Chassis : public ossian::IODataBuilder<std::mutex, ChassisMotorsModel>
{
public:
	//俯视，左前，左后，右后，右前，逆时针
	enum MotorPosition
	{
		LF = 0,
		LR,
		RR,
		RF
	};

	OSSIAN_SERVICE_SETUP(Chassis(
		ossian::MotorManager* motorManager,
		ossian::IOData<ChassisMotorsModel>* ioData))
		: m_MotorManager(motorManager)
		, m_MotorsStatus()
		, m_IOData(ioData)
	{
		m_MotorMsgCheck.fill(false);
	}

	auto AddMotor(const MotorPosition position,
				  const std::string location,
				  const unsigned int motorId,
				  const unsigned int writerCanId) -> void
	{
		m_Motors[position] =
			m_MotorManager->AddMotor<ossian::DJIMotor3508Mt>(
				location,
				m_MotorManager->GetOrAddWriter<ossian::DJIMotor3508WriterMt>(location, writerCanId),
				[this, position](const std::shared_ptr<ossian::DJIMotor3508Mt>& motor)
				{
					MotorReceiveProc(motor, position);
				},
				motorId);
	}

	auto MotorReceiveProc(const std::shared_ptr<ossian::DJIMotor3508Mt>& motor,
						  MotorPosition position) -> void
	{
		const auto status = motor->GetRef();
		motor->Lock();
		m_MotorsStatus.m_RPM[position] = status.m_RPM;
		m_MotorsStatus.m_Encoding[position] = status.m_Encoding;
		motor->UnLock();

		m_MotorMsgCheck[position] = true;
		if (!(m_MotorMsgCheck[LF] && m_MotorMsgCheck[LR] && m_MotorMsgCheck[RR] && m_MotorMsgCheck[RF]))
		{
			return;
		}

		m_IOData->Set(m_MotorsStatus);

		m_MotorMsgCheck.fill(false);
	}

	void SendCurrentToMotors(const std::array<double, kNumChassisMotors>& currentSend)
	{
		for (size_t i = 0; i < kNumChassisMotors; ++i)
		{
			m_Motors[i]->SetVoltage(currentSend[i]);
		}
		m_Motors[LR]->Writer()->PackAndSend();
	}

private:
	ossian::MotorManager* m_MotorManager;
	ChassisMotorsModel m_MotorsStatus;
	ossian::IOData<ChassisMotorsModel>* m_IOData;
	std::array<std::shared_ptr<ossian::DJIMotor3508Mt>, kNumChassisMotors> m_Motors;
	std::array<bool, kNumChassisMotors> m_MotorMsgCheck;
};

class ChassisCtrlTask : public ossian::IExecutable
{
public:
	//麦轮运动
	static constexpr double kWheelRadius = 76.0 / 1000.0;  ///< m
	static constexpr double kWheelXn = 175.0 / 1000.0; ///< m
	static constexpr double kWheelYn = 232.5 / 1000;   ///< m
	static constexpr double kWheelSpeedLimit = 400;            ///< 单个麦轮的最大转速rpm
	static constexpr double kWheelSpeedToMotorRPMCoef = 11.875;
	//static constexpr double CHASSIS_MOTOR_RPM_TO_VECTOR_SEN = 0.000415809748903494517209f;

	//底盘功率控制
	static constexpr double kBufferTotalCurrentLimit = 45000;
	static constexpr double kPowerTotalCurrentLimit = 50000;
	static constexpr double kSpCapWarnVoltage = 12;

	//遥控器解析
	static constexpr size_t kChassisXChannel = 1; ///< 控制底盘 前后 速度的遥控器通道
	static constexpr size_t kChassisYChannel = 0; ///< 控制底盘 左右 速度的遥控器通道
	static constexpr size_t kChassisZChannel = 2; ///< 控制底盘 旋转 速度的遥控器通道 仅当使用Openloop_Z模式时可用
	static constexpr size_t kChassisModeChannel = 0; ///< 选择底盘状态的开关通道

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	static constexpr int16_t kChassisRCDeadband = 10;     ///< 摇杆死区
	static constexpr double kChassisVxRCSen = -0.006;  ///< 遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例0.006
	static constexpr double kChassisVyRCSen = 0.005; ///< 遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
	static constexpr double kChassisWzRCSen = 0.01;  ///< 不跟随云台的时候，遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
	static constexpr double kChassisCtrlPeriod = 0.012;  //底盘控制周期s，用于低通滤波器

	//底盘运动
	static constexpr double kChassisVxLimit = 4.5; ///< m/s
	static constexpr double kChassisVyLimit = 3.5; ///< m/s
	static double kTopWz;                          ///< 底盘陀螺旋转速度 rad/s

	//pid参数 [TODO]底盘旋转角速度闭环
	static std::array<double, 5> PIDWheelSpeedParams;
	static std::array<double, 5> PIDChassisAngleParams;

	static double kVxFilterCoef, kVyFilterCoef;
	static double kRPMFdbFilterCoef;

	//俯视，左前，左后，右后，右前，逆时针
	enum MotorPosition
	{
		LF = 0,
		LR,
		RR,
		RF
	};

	enum ChassisMode
	{
		Disable, ///< 失能
		Follow_Gimbal_Yaw, ///< 跟随云台

		//Follow_Chassis_Yaw,		 ///< 遥控器控制底盘旋转，底盘角速度闭环。工程采用。

		Top, ///< 小陀螺
		Openloop_Z ///< 单独调试底盘
	};

	OSSIAN_SERVICE_SETUP(ChassisCtrlTask(ossian::IOData<ChassisMotorsModel>* motors,
										 ossian::IOData<RemoteStatus>* remote,
										 ICapacitor* capacitor,
										 Chassis* chassis,
										 GimbalCtrlTask* gimbalCtrlTask,
										 Utils::ConfigLoader* config,
										 ossian::IOData<PowerHeatData>* powerHeatDataListener))

		: m_MotorsListener(motors)
		, m_RCListener(remote)
		, m_SpCap(capacitor)
		, m_Chassis(chassis)
		, m_GimbalCtrlTask(gimbalCtrlTask)
		, m_Config(config)
		, m_RefereePowerHeatDataListener(powerHeatDataListener)
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
		PIDChassisAngleParams[4] = m_Config->Instance<Configuration>()->mutable_pidchassisangle()->thiout();

		kTopWz = m_Config->Instance<Configuration>()->mutable_chassis()->ktopwz();
		kVxFilterCoef = m_Config->Instance<Configuration>()->mutable_chassis()->kvxfiltercoef();
		kVyFilterCoef = m_Config->Instance<Configuration>()->mutable_chassis()->kvyfiltercoef();
		kRPMFdbFilterCoef = m_Config->Instance<Configuration>()->mutable_chassis()->krpmfdbfiltercoef();

		double coef = kWheelXn + kWheelYn;
		m_WheelKinematicMat << 1, -1, -coef,
			1, 1, -coef,
			-1, 1, -coef,
			-1, -1, -coef; //3,4号电机转向与轮子相反
		/*m_WheelKinematicMat << 1, -1, -coef,
			1, 1, -coef,
			1, -1, coef,
			1, 1, coef;*/

		m_FlagInitChassis = true;

		m_RCInputFilters[0].SetState(kVxFilterCoef, kChassisCtrlPeriod);
		m_RCInputFilters[1].SetState(kVyFilterCoef, kChassisCtrlPeriod);
		FirstOrderFilter rpmFdbFilter(kRPMFdbFilterCoef, kChassisCtrlPeriod);
		m_RPMFdbFilters.fill(rpmFdbFilter);

		PIDController pidWheelSpeed;
		pidWheelSpeed.SetParams(PIDWheelSpeedParams);
		m_PIDChassisSpeed.fill(pidWheelSpeed);

		m_PIDChassisAngle.SetParams(PIDChassisAngleParams);
		m_PIDChassisAngle.SetFlagAngleLoop();
		/*m_RCListener->AddOnChange([](const RemoteStatus& value) {
			SPDLOG_INFO("@RemoteData=[$ch0={},$ch1={},$ch2={},$ch3={},$ch4={}]",
				value.ch[0], value.ch[1], value.ch[2], value.ch[3], value.ch[4]);});

		m_RefereePowerHeatDataListener->AddOnChange([](const PowerHeatData& value) {
			SPDLOG_INFO("@RefereePowerHeatData=[$ChassisPower={},$ChassisPowerBuffer={},$MaxPower={}]",
				value.m_ChassisPower,
				value.m_ChassisPowerBuffer,
				80); });*/
	}

	void InitChassis()
	{
		m_AngleSet = 0;

		std::for_each(m_RCInputFilters.begin(), m_RCInputFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });
		std::for_each(m_RPMFdbFilters.begin(), m_RPMFdbFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });

		std::for_each(m_PIDChassisSpeed.begin(), m_PIDChassisSpeed.end(), [](PIDController& x) { x.Reset(); });
		m_PIDChassisAngle.Reset();

		m_FlagInitChassis = false;
	}

	void UpdateChassisSensorFeedback()
	{
		m_ChassisSensorValues.rc = m_RCListener->Get();
		//SPDLOG_INFO("@RemoteData=[$ch0={},$ch1={},$ch2={},$ch3={},$ch4={}]", m_ChassisSensorValues.rc.ch[0], m_ChassisSensorValues.rc.ch[1], m_ChassisSensorValues.rc.ch[2], m_ChassisSensorValues.rc.ch[3], m_ChassisSensorValues.rc.ch[4]);
		//m_ChassisSensorValues.spCap = m_SpCap->Get();
		m_ChassisSensorValues.gimbalInputSrc = m_GimbalCtrlTask->GimbalCtrlSrc();
		m_ChassisSensorValues.relativeAngle = m_GimbalCtrlTask->RelativeAngleToChassis();

		m_ChassisSensorValues.refereePowerHeatData = m_RefereePowerHeatDataListener->Get();
		m_ChassisSensorValues.refereePowerHeatData.m_ChassisVolt /= 1000; //v
		/*SPDLOG_INFO("@RefereePowerHeatData=[$ChassisPower={},$ChassisPowerBuffer={},$MaxPower={}]",
			m_ChassisSensorValues.refereePowerHeatData.m_ChassisPower,
			m_ChassisSensorValues.refereePowerHeatData.m_ChassisPowerBuffer,
			80);*/
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

	auto ExecuteProc() -> void override
	{
		using Clock = std::chrono::high_resolution_clock;
		using TimeStamp = Clock::time_point;
		
		TimeStamp lastTime = Clock::now();
		while (true)
		{
			while (12000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			/*SPDLOG_INFO("@Interval=[$t={}]",
						std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count() / 1000.0);*/
			
			lastTime = Clock::now();
			m_MotorsStatus = m_MotorsListener->Get();

			UpdateChassisSensorFeedback();

			if (m_FlagInitChassis)
				InitChassis();

			ChassisModeSet();
			//[TODO] 模式切换过渡

			ChassisExpAxisSpeedSet();
			ChassisCtrl();

			/*static hrClock::time_point lastSendSpCapTimestamp;
			long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
				hrClock::now() - lastSendSpCapTimestamp).count();
			if (interval > 100)   //不推荐以太高的频率发送功率数据，推荐10Hz
			{
				m_SpCap->SetPower(m_ChassisSensorValues.refereePowerHeatData.m_ChassisPower);
				lastSendSpCapTimestamp = hrClock::now();
			}*/
		}
	}

private:
	Utils::ConfigLoader* m_Config;
	ossian::IOData<RemoteStatus>* m_RCListener; //遥控器
	ossian::IOData<ChassisMotorsModel>* m_MotorsListener;
	ICapacitor* m_SpCap;
	
	Chassis* m_Chassis;
	GimbalCtrlTask* m_GimbalCtrlTask;
	ossian::IOData<PowerHeatData>* m_RefereePowerHeatDataListener;

	bool m_FlagInitChassis;
	ChassisMotorsModel m_MotorsStatus;

	struct ChassisSensorFeedback
	{
		RemoteStatus rc;
		double gyroX, gyroY, gyroZ, gyroSpeedX, gyroSpeedY, gyroSpeedZ;
		///< 底盘imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		CapacitorStatus spCap;              ///< 超级电容数据
		PowerHeatData refereePowerHeatData; ///< 裁判系统数据
		int refereeMaxPwr = 80, refereeMaxBuf = 60;
		double relativeAngle; ///< 底盘坐标系与云台坐标系的夹角 当前yaw编码值减去中值 rad
		GimbalCtrlTask::GimbalInputSrc gimbalInputSrc;
	} m_ChassisSensorValues;

	double m_VxSet, m_VySet, m_WzSet; //三轴速度期望
	double m_AngleSet;                //底盘角度目标值


	ChassisMode m_CurChassisMode;
	Eigen::Vector4d m_WheelSpeedSet;
	Eigen::Matrix<double, 4, 3> m_WheelKinematicMat;
	std::array<double, kNumChassisMotors> m_CurrentSend;

	std::array<FirstOrderFilter, 2> m_RCInputFilters;
	std::array<FirstOrderFilter, kNumChassisMotors> m_RPMFdbFilters;
	PIDController m_PIDChassisAngle;                                ///< 底盘要旋转的角度--->底盘旋转角速度  底盘跟随角度环
	std::array<PIDController, kNumChassisMotors> m_PIDChassisSpeed; ///< 麦轮转速--->3508电流
};

#endif // OSSIAN_CHASSIS_HPP
