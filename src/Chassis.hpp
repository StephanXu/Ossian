#ifndef OSSIAN_CHASSIS_HPP
#define OSSIAN_CHASSIS_HPP

#include <ossian/motors/Motor.hpp>
#include <ossian/motors/DJIMotor.hpp>
#include <ossian/IOData.hpp>
#include <ossian/Pipeline.hpp>
#include <Config.schema.hpp>
#include <ossian/Configuration.hpp>
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
		ossian::IOData<ChassisMotorsModel>* ioData,
		CapacitorMt* spCap))
		: m_MotorManager(motorManager)
		, m_MotorsStatus()
		, m_IOData(ioData)
		, m_SpCap(spCap)
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
		m_MotorsStatus.m_Current[position] = status.m_Current;
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

	void SetCapPwr(const double power)
	{
		static std::chrono::high_resolution_clock::time_point lastSendSpCapTimestamp 
			= std::chrono::high_resolution_clock::now();
		long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - lastSendSpCapTimestamp).count();
		if (interval > 100)   //不推荐以太高的频率发送功率数据，推荐10Hz
		{
			m_SpCap->SetPower(power);
			lastSendSpCapTimestamp = std::chrono::high_resolution_clock::now();
		}
		
	}

private:
	ossian::MotorManager* m_MotorManager;
	CapacitorMt* m_SpCap;
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
	static constexpr double kWheelSpeedLimit = 300;            ///< 单个麦轮的最大转速rpm
	static constexpr double kWheelSpeedToMotorRPMCoef = 3591 / 187.0;
	static constexpr double kDegreeToRadCoef = M_PI / 180.0;
	//static constexpr double CHASSIS_MOTOR_RPM_TO_VECTOR_SEN = 0.000415809748903494517209f;

	//底盘功率控制
	static constexpr double kBufferTotalCurrentLimit = 45000;
	static constexpr double kPowerTotalCurrentLimit = 50000;
	static constexpr double kSpCapWarnVoltage = 13;

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
	static constexpr double kChassisVyRCSen = -0.005; ///< 遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
	static constexpr double kChassisWzRCSen = 0.01;  ///< 不跟随云台的时候，遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
	static constexpr double kChassisAngleWzRCSen = 0.000005; ///< 跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
	static constexpr double kChassisCtrlPeriod = 0.002;  //底盘控制周期s，用于低通滤波器

	//底盘运动
	static constexpr double kChassisVxLimit = 3; ///< m/s
	static constexpr double kChassisVyLimit = 1.5; ///< m/s
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
		Init,	 ///< 初始化
		Follow_Gimbal_Yaw, ///< 跟随云台

		Follow_Chassis_Yaw,		 ///< 遥控器控制底盘旋转，底盘角速度闭环。工程采用。

		Top, ///< 小陀螺
		Openloop_Z ///< 单独调试底盘
	};

	OSSIAN_SERVICE_SETUP(ChassisCtrlTask(ossian::IOData<ChassisMotorsModel>* motors,
										 ossian::IOData<RemoteStatus>* remote,
										 ossian::IOData<CapacitorStatus>* capacitorListener,
										 Chassis* chassis,
										 ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
										 ossian::IOData<PowerHeatData>* powerHeatDataListener,
										 ossian::IOData<RobotStatus>* robotStatusListener,
										 ossian::IOData<GimbalStatus>* gimbalStatusListener,
										 ossian::IOData<GyroA110Status<GyroType::Chassis>>* gyroListener))

		: m_MotorsListener(motors)
		, m_RCListener(remote)
		, m_SpCapListener(capacitorListener)
		, m_Chassis(chassis)
		, m_Config(config)
		, m_RefereePowerHeatDataListener(powerHeatDataListener)
		, m_RefereeRobotStatusListener(robotStatusListener)
		, m_GimbalStatusListener(gimbalStatusListener)
		, m_GyroListener(gyroListener)
	{
		PIDWheelSpeedParams[0] = *m_Config->Instance()->pids->pidWheelSpeed->kP;
		PIDWheelSpeedParams[1] = *m_Config->Instance()->pids->pidWheelSpeed->kI;
		PIDWheelSpeedParams[2] = *m_Config->Instance()->pids->pidWheelSpeed->kD;
		PIDWheelSpeedParams[3] = *m_Config->Instance()->pids->pidWheelSpeed->thOut;
		PIDWheelSpeedParams[4] = *m_Config->Instance()->pids->pidWheelSpeed->thIOut;

		PIDChassisAngleParams[0] = *m_Config->Instance()->pids->pidChassisAngle->kP;
		PIDChassisAngleParams[1] = *m_Config->Instance()->pids->pidChassisAngle->kI;
		PIDChassisAngleParams[2] = *m_Config->Instance()->pids->pidChassisAngle->kD;
		PIDChassisAngleParams[3] = *m_Config->Instance()->pids->pidChassisAngle->thOut;
		PIDChassisAngleParams[4] = *m_Config->Instance()->pids->pidChassisAngle->thIOut;

		kTopWz = *m_Config->Instance()->control->chassis->topWz;
		kVxFilterCoef = *m_Config->Instance()->control->chassis->vxFilterCoef;
		kVyFilterCoef = *m_Config->Instance()->control->chassis->vyFilterCoef;
		kRPMFdbFilterCoef = *m_Config->Instance()->control->chassis->rpmFdbFilterCoef;

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
		m_TopWzFilter.SetState(0.9, kChassisCtrlPeriod);

		PIDController pidWheelSpeed;
		pidWheelSpeed.SetParams(PIDWheelSpeedParams);
		m_PIDChassisSpeed.fill(pidWheelSpeed);

		m_PIDChassisAngle.SetParams(PIDChassisAngleParams);
		m_PIDChassisAngle.SetFlagAngleLoop();

		/*m_RCListener->AddOnChange([](const RemoteStatus& value) {
			SPDLOG_INFO("@RemoteData=[$ch0={},$ch1={},$ch2={},$ch3={},$ch4={},$sw0={},$sw1={}]",
				value.ch[0], value.ch[1], value.ch[2], value.ch[3], value.ch[4],value.sw[0], value.sw[1]);});*/
		/*m_RCListener->AddOnChange([](const RemoteStatus& value) {
			SPDLOG_INFO("@KeyMouseData=[$X={},$Y={},$Z={},$ClickLeft={},$ClickRight={},$Keyboard={}]",
				value.mouse[0], value.mouse[1], value.mouse[2], (int)value.click[0], (int)value.click[1],value.keyboard);});*/

		/*m_RefereePowerHeatDataListener->AddOnChange([](const PowerHeatData& value) {
			SPDLOG_INFO("@RefereePowerHeatData=[$ChassisPower={},$ChassisPowerBuffer={}]",
				value.m_ChassisPower,
				value.m_ChassisPowerBuffer); 
		});
		m_RefereeRobotStatusListener->AddOnChange([](const RobotStatus& value) {
			SPDLOG_INFO("@ChassisPowerLimit=[$MaxPower={}]",
				value.m_ChassisMaxPower);
		});
		m_SpCapListener->AddOnChange([](const CapacitorStatus& value) {
			SPDLOG_INFO("@SpCap=[$InputV={},$CapV={},$InputI={},$TargetP={}]",
				value.m_InputVoltage, value.m_CapacitorVoltage, value.m_TestCurrent, value.m_TargetPower);
		});*/

		/*m_GyroListener->AddOnChange([](const GyroA110Status<GyroType::Chassis>& value) {
			SPDLOG_INFO("@ChassisImu=[$yaw={},$yawSpeed={}]", value.m_Yaw, value.m_ZAngleSpeed);
		});*/
	}

	void InitChassis()
	{
		static std::chrono::high_resolution_clock::time_point timestampInit;
		static bool flagStartInit = true;
		if (flagStartInit)
		{
			timestampInit = std::chrono::high_resolution_clock::now();
			flagStartInit = false;
		}
			
		long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - timestampInit).count();

		if (interval > 2000 && timestampInit != std::chrono::high_resolution_clock::time_point()) //待imu稳定后再读数
		{
			m_AngleSet = m_ChassisSensorValues.imu.m_Yaw;

			std::for_each(m_RCInputFilters.begin(), m_RCInputFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });
			std::for_each(m_RPMFdbFilters.begin(), m_RPMFdbFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });

			std::for_each(m_PIDChassisSpeed.begin(), m_PIDChassisSpeed.end(), [](PIDController& x) { x.Reset(); });
			m_PIDChassisAngle.Reset();

			m_FlagInitChassis = false;
			flagStartInit = true;
		}
		
	}

	void UpdateChassisSensorFeedback()
	{
		m_MotorsStatus = m_MotorsListener->Get();
		m_ChassisSensorValues.rc = m_RCListener->Get();
		m_ChassisSensorValues.spCap = m_SpCapListener->Get();
		m_ChassisSensorValues.gimbalStatus = m_GimbalStatusListener->Get();

		m_ChassisSensorValues.imu = m_GyroListener->Get();
		m_ChassisSensorValues.imu.m_XAxisSpeed *= kDegreeToRadCoef;
		m_ChassisSensorValues.imu.m_Roll *= kDegreeToRadCoef;
		m_ChassisSensorValues.imu.m_YAxisSpeed *= kDegreeToRadCoef;
		m_ChassisSensorValues.imu.m_Pitch *= kDegreeToRadCoef;
		m_ChassisSensorValues.imu.m_ZAxisSpeed *= kDegreeToRadCoef;
		m_ChassisSensorValues.imu.m_Yaw *= kDegreeToRadCoef;
		//m_ChassisSensorValues.gimbalCtrlMode = m_GimbalCtrlTask->GimbalCtrlSrc();

		m_ChassisSensorValues.refereePowerHeatData = m_RefereePowerHeatDataListener->Get();
		m_ChassisSensorValues.refereePowerHeatData.m_ChassisVolt /= 1000.0; //v

		m_ChassisSensorValues.refereeRobotStatus = m_RefereeRobotStatusListener->Get();
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
			while (2000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			/*SPDLOG_INFO("@Interval=[$t={}]",
						std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count() / 1000.0);*/

			lastTime = Clock::now();
			

			UpdateChassisSensorFeedback();

			if (m_FlagInitChassis)
				InitChassis();

			ChassisModeSet();
			//[TODO] 模式切换过渡

			ChassisExpAxisSpeedSet();
			ChassisCtrl();

			m_Chassis->SetCapPwr(m_ChassisSensorValues.refereeRobotStatus.m_ChassisMaxPower);
		}
	}

private:
	ossian::Utils::ConfigLoader<Config::ConfigSchema>* m_Config;
	ossian::IOData<RemoteStatus>* m_RCListener; //遥控器
	ossian::IOData<ChassisMotorsModel>* m_MotorsListener;
	ossian::IOData<CapacitorStatus>* m_SpCapListener;
	
	Chassis* m_Chassis;
	ossian::IOData<PowerHeatData>* m_RefereePowerHeatDataListener;
	ossian::IOData<RobotStatus>* m_RefereeRobotStatusListener;

	bool m_FlagInitChassis;
	ChassisMotorsModel m_MotorsStatus;

	struct ChassisSensorFeedback
	{
		RemoteStatus rc;
		GyroA110Status<GyroType::Chassis> imu;
		///< 底盘imu数据 [TODO] gyroSpeedZ = cos(pitch) * gyroSpeedZ - sin(pitch) * gyroSpeedX
		CapacitorStatus spCap;              ///< 超级电容数据
		PowerHeatData refereePowerHeatData; ///< 裁判系统数据
		RobotStatus refereeRobotStatus;
		GimbalStatus gimbalStatus;
	} m_ChassisSensorValues;

	double m_VxSet, m_VySet, m_WzSet; //三轴速度期望
	double m_AngleSet;                //底盘角度目标值


	ChassisMode m_CurChassisMode;
	ossian::IOData<GimbalStatus>* m_GimbalStatusListener;
	ossian::IOData<GyroA110Status<GyroType::Chassis>>* m_GyroListener;
	Eigen::Vector4d m_WheelSpeedSet;
	Eigen::Matrix<double, 4, 3> m_WheelKinematicMat;
	std::array<double, kNumChassisMotors> m_CurrentSend;

	std::array<FirstOrderFilter, 2> m_RCInputFilters;
	std::array<FirstOrderFilter, kNumChassisMotors> m_RPMFdbFilters;
	FirstOrderFilter m_TopWzFilter;
	PIDController m_PIDChassisAngle;                                ///< 底盘要旋转的角度--->底盘旋转角速度  底盘跟随角度环
	std::array<PIDController, kNumChassisMotors> m_PIDChassisSpeed; ///< 麦轮转速--->3508电流
};

#endif // OSSIAN_CHASSIS_HPP
