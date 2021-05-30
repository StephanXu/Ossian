#ifndef OSSIAN_GUN_HPP
#define OSSIAN_GUN_HPP

#include <ossian/motors/Motor.hpp>
#include <ossian/motors/DJIMotor.hpp>
#include <ossian/IOData.hpp>

#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gimbal.hpp"
#include "Referee.hpp"
#include <chrono>
#include <array>
#include <atomic>
#include <ctime>
#include <cmath>

//constexpr size_t kNumGunFricMotorsFake = 3;
constexpr size_t kNumGunFricMotors = 2; 
constexpr size_t kNumGunFeedMotors = 1;

OSSIAN_MULTIPLE_MOTORS_STATUS(GunFricMotorsModel, kNumGunFricMotors);
OSSIAN_MULTIPLE_MOTORS_STATUS(GunFeedMotorsModel, kNumGunFeedMotors);

class Gun : public ossian::IODataBuilder<std::mutex, GunFricMotorsModel, GunFeedMotorsModel>
{
public:
	
	enum MotorPosition
	{
		FricBelow = 0, FricUpper, Feed
	};

	OSSIAN_SERVICE_SETUP(Gun(
		ossian::MotorManager*			    motorManager,
		ossian::IOData<GunFricMotorsModel>* fricMotorsData,
		ossian::IOData<GunFeedMotorsModel>* feedMotorsData))
		: m_MotorManager(motorManager)
		, m_FricMotorsStatus()
		, m_FeedMotorsStatus()
		, m_FricMotorsData(fricMotorsData)
		, m_FeedMotorsData(feedMotorsData)
	{
		m_FricMotorMsgCheck.fill(false);
	}

	auto AddMotor(MotorPosition position,
		const std::string location,
		const unsigned int motorId,
		const unsigned int writerCanId)->void
	{
		if (position == FricBelow || position == FricUpper) {
			m_MotorsFric[position] =
				m_MotorManager->AddMotor<ossian::DJIMotor3508Mt>(
					location,
					m_MotorManager->GetOrAddWriter<ossian::DJIMotor3508WriterMt>(location, writerCanId),
					[this, position](const std::shared_ptr<ossian::DJIMotor3508Mt>& motor)
					{
						MotorFricReceiveProc(motor, position);
					},
					motorId);
		}
		else if (position == Feed)
			m_MotorFeed =
				m_MotorManager->AddMotor<ossian::DJIMotor2006Mt>(
					location,
					m_MotorManager->GetOrAddWriter<ossian::DJIMotor2006WriterMt>(location, writerCanId),
					[this, position](const std::shared_ptr<ossian::DJIMotor2006Mt>& motor)
					{
						MotorFeedReceiveProc(motor, position);
					},
					motorId);
	}

	auto MotorFricReceiveProc(const std::shared_ptr<ossian::DJIMotor3508Mt>& motor, MotorPosition position)->void
	{
		const auto status = motor->GetRef();
		motor->Lock();
		m_FricMotorsStatus.m_RPM[position] = status.m_RPM;
		m_FricMotorsStatus.m_Encoding[position] = status.m_Encoding;
		motor->UnLock();

		m_FricMotorMsgCheck[position] = true;
		if (!(m_FricMotorMsgCheck[FricBelow] && m_FricMotorMsgCheck[FricUpper]))
		{
			return;
		}
		m_FricMotorsData->Set(m_FricMotorsStatus);

		m_FricMotorMsgCheck.fill(false);
	}

	auto MotorFeedReceiveProc(const std::shared_ptr<ossian::DJIMotor2006Mt>& motor, MotorPosition position)->void
	{
		const auto status = motor->GetRef();
		motor->Lock();
		m_FeedMotorsStatus.m_RPM[0] = status.m_RPM;
		m_FeedMotorsStatus.m_Encoding[0] = status.m_Encoding;
		motor->UnLock();
		
		m_FeedMotorsData->Set(m_FeedMotorsStatus);
	}
	
	void SendCurrentToMotorsFric(const std::array<double, kNumGunFricMotors>& currentSend)
	{
		for (size_t i = 0; i < kNumGunFricMotors; ++i)
		{
			m_MotorsFric[i]->SetVoltage(currentSend[i]);
		}
		m_MotorsFric[FricBelow]->Writer()->PackAndSend();
	}

	void SendCurrentToMotorFeed(const double& currentSend)
	{
		m_MotorFeed->SetVoltage(currentSend);
		m_MotorFeed->Writer()->PackAndSend();
	}

private:
	ossian::MotorManager* m_MotorManager;
	GunFricMotorsModel m_FricMotorsStatus;
	ossian::IOData<GunFricMotorsModel>* m_FricMotorsData;
	GunFeedMotorsModel m_FeedMotorsStatus;
	ossian::IOData<GunFeedMotorsModel>* m_FeedMotorsData;

	std::array<std::shared_ptr<ossian::DJIMotor3508Mt>, 2> m_MotorsFric;
	std::shared_ptr<ossian::DJIMotor2006Mt> m_MotorFeed;
	std::array<bool, 2> m_FricMotorMsgCheck;

	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	
};

enum class FricMode
{
	Disable, Enable
};

struct FricStatus
{
	FricMode m_Mode;
	bool m_FlagLowRPM;
};

class FricCtrlTask : public ossian::IExecutable, public ossian::IODataBuilder<std::mutex, FricStatus>
{
public:
	//摩擦轮转速期望rpm [TODO]实验得出不同等级下的射击初速度上限所对应的摩擦轮转速期望
	static int16_t kFricSpeed15;
	static int16_t kFricSpeed18;
	static int16_t kFricSpeed22;
	static int16_t kFricSpeed30;

	//遥控器解析
	static constexpr int16_t kGunRCDeadband = 50; //拨轮死区
	static constexpr size_t kShootModeChannel = 1;

	//pid参数
	static std::array<double, 5> PIDFricSpeedParams;

	//摩擦轮转速下限
	static constexpr int16_t kFricLowSpeed = 300;

	enum MotorPosition
	{
		FricBelow = 0, FricUpper
	};
	
	OSSIAN_SERVICE_SETUP(FricCtrlTask(ossian::IOData<RemoteStatus>* remote,
		ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		Gun* gun,
		ossian::IOData<GunFricMotorsModel>* motorsFricListener,
		ossian::IOData<GimbalStatus>* gimbalStatusListener,
		ossian::IOData<FricStatus>* fricStatusSender,
		ossian::IOData<RobotStatus>* robotStatusListener))
		: m_RCListener(remote)
		, m_Config(config)
		, m_Gun(gun)
		, m_MotorsFricListener(motorsFricListener)
		, m_GimbalStatusListener(gimbalStatusListener)
		, m_FricStatusSender(fricStatusSender)
		, m_RefereeRobotStatusListener(robotStatusListener)
	{
		PIDFricSpeedParams[0] = *m_Config->Instance()->pids->pidFricSpeed->kP;
		PIDFricSpeedParams[1] = *m_Config->Instance()->pids->pidFricSpeed->kI;
		PIDFricSpeedParams[2] = *m_Config->Instance()->pids->pidFricSpeed->kD;
		PIDFricSpeedParams[3] = *m_Config->Instance()->pids->pidFricSpeed->thOut;
		PIDFricSpeedParams[4] = *m_Config->Instance()->pids->pidFricSpeed->thIOut;

		kFricSpeed15 = *m_Config->Instance()->control->gun->fricSpeed15;
		kFricSpeed18 = *m_Config->Instance()->control->gun->fricSpeed18;
		kFricSpeed22 = *m_Config->Instance()->control->gun->fricSpeed22;
		kFricSpeed30 = *m_Config->Instance()->control->gun->fricSpeed30;

		m_FlagInitFric = true;
		m_CurFricMode = FricMode::Disable;

		PIDController pidFricSpeed;
		pidFricSpeed.SetParams(PIDFricSpeedParams);
		m_PIDFricSpeed.fill(pidFricSpeed);

		FirstOrderFilter rpmFdbFilter(0.25, 0.003);
		m_RPMFdbFilters.fill(rpmFdbFilter);
	}

	void InitFric()
	{
		m_CurFricMode = m_LastFricMode = FricMode::Disable;

		std::for_each(m_PIDFricSpeed.begin(), m_PIDFricSpeed.end(), [](PIDController& x) {x.Reset(); });
		std::for_each(m_RPMFdbFilters.begin(), m_RPMFdbFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });

		m_FlagInitFric = false;
	}

	void UpdateFricSensorFeedback()
	{
		m_FricMotorsStatus = m_MotorsFricListener->Get();
		m_FricSensorValues.rc = m_RCListener->Get();
		m_FricSensorValues.keyboardMode = (m_FricSensorValues.rc.sw[0] == kRCSwUp && m_FricSensorValues.rc.sw[1] == kRCSwUp);
		m_FricSensorValues.gimbalStatus = m_GimbalStatusListener->Get();
		m_FricSensorValues.refereeRobotStatus = m_RefereeRobotStatusListener->Get();

		//m_FricSensorValues.gimbalCtrlMode = m_GimbalCtrlTask->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理
	}

	/*bool Stopped() 
	{ 
		return (m_CurFricMode == FricMode::Disable
			|| std::abs(m_FricMotorsStatus.m_RPM[FricBelow]) < kFricLowSpeed
			|| std::abs(m_FricMotorsStatus.m_RPM[FricUpper]) < kFricLowSpeed);
	}*/

	void FricModeSet();

	void FricExpSpeedSet();

	void FricCtrl();

	auto ExecuteProc() -> void override 
	{
		using Clock = std::chrono::high_resolution_clock;
		using TimeStamp = Clock::time_point;

		TimeStamp lastTime = Clock::now();
		while (true)
		{
			//SPDLOG_TRACE("@FricInterval=[$timefric={}]", std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count());
			while (1000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();


			UpdateFricSensorFeedback();
			if (m_FlagInitFric)
				InitFric();

			FricModeSet();
			m_Status.m_Mode = m_CurFricMode;
			m_Status.m_FlagLowRPM = (std::abs(m_FricMotorsStatus.m_RPM[FricBelow]) < kFricLowSpeed
									 || std::abs(m_FricMotorsStatus.m_RPM[FricUpper]) < kFricLowSpeed);
			m_FricStatusSender->Set(m_Status);
			FricExpSpeedSet();
			FricCtrl();
		}
	}

private:
	ossian::Utils::ConfigLoader<Config::ConfigSchema>* m_Config;
	ossian::IOData<RemoteStatus>* m_RCListener;  //遥控器

	ossian::IOData<GunFricMotorsModel>* m_MotorsFricListener;
	ossian::IOData<GimbalStatus>* m_GimbalStatusListener;
	ossian::IOData<RobotStatus>* m_RefereeRobotStatusListener;

	FricStatus m_Status;
	ossian::IOData<FricStatus>* m_FricStatusSender;

	GunFricMotorsModel m_FricMotorsStatus;
	Gun* m_Gun;

	struct FricSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		GimbalStatus gimbalStatus;
		//double refereeCurHeat, refereeHeatLimit;
		RobotStatus refereeRobotStatus;
		bool keyboardMode;

	} m_FricSensorValues;

	bool m_FlagInitFric;
	FricMode m_CurFricMode, m_LastFricMode;

	int16_t m_FricSpeedSet;
	std::array<PIDController, 2> m_PIDFricSpeed;
	std::array<FirstOrderFilter, kNumGunFricMotors> m_RPMFdbFilters;
};

class FeedCtrlTask : public ossian::IExecutable
{
public:
	enum MotorPosition
	{
		Feed = 0
	};

	enum FeedMode
	{
		Disable, Stop, Reload, Reverse, Semi, Burst, Auto, Ready
	};

	//遥控器解析
	static constexpr int16_t kGunRCDeadband = 100; //拨轮死区
	static constexpr size_t kShootModeChannel = 1; //右开关1, 拨轮4

	//枪口热量机制
	static constexpr int kHeatPerBullet = 10;

	//拨弹轮
	static constexpr double kSpeedToMotorRPMCoef = 36;
	static constexpr int kBurstBulletNum = 3; //点射的子弹发射数量
	static constexpr int16_t kFeedJamRPM = 5;     //判断卡弹的拨弹轮转速阈值
	static constexpr double kMotorEcdToRadCoef = 2 * M_PI / 8192.0 / kSpeedToMotorRPMCoef;  //电机编码值---拨盘旋转角度
	static constexpr int kNumCells = 8;
	static constexpr double kDeltaAnglePerBullet = 2.0 * M_PI / kNumCells;  //拨弹轮逆时针旋转

	static int16_t kFeedSemiSpeed;   //单发时，拨弹轮供弹的转速
	static int16_t kFeedBurstSpeed;  //点射时，拨弹轮供弹的转速
	static int16_t kFeedAutoSpeed;   //连发时，拨弹轮供弹的转速   点射>连发>单发

	//pid参数
	static std::array<double, 5> PIDFeedAngleParams;
	static std::array<double, 5> PIDFeedSpeedParams;

	OSSIAN_SERVICE_SETUP(FeedCtrlTask(ossian::IOData<RemoteStatus>* remote,
		ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		Gun* gun,
		ossian::IOData<GunFeedMotorsModel>* motorFeedListener,
		ossian::IOData<PowerHeatData>* powerHeatDataListener,
		ossian::IOData<RobotStatus>* robotStatusListener,
		ossian::IOData<ShootData>* shootDataListener,
		ossian::IOData<GimbalStatus>* gimbalStatusListener,
		ossian::IOData<FricStatus>* fricStatusListener))
		: m_RCListener(remote)
		, m_Config(config)
		, m_Gun(gun)
		, m_MotorFeedListener(motorFeedListener)
		, m_RefereePowerHeatDataListener(powerHeatDataListener)
		, m_RefereeRobotStatusListener(robotStatusListener)
		, m_RefereeShootDataListener(shootDataListener)
		, m_GimbalStatusListener(gimbalStatusListener)
		, m_FricStatusListener(fricStatusListener)
	{
		PIDFeedAngleParams[0] = *m_Config->Instance()->pids->pidFeedAngle->kP;
		PIDFeedAngleParams[1] = *m_Config->Instance()->pids->pidFeedAngle->kI;
		PIDFeedAngleParams[2] = *m_Config->Instance()->pids->pidFeedAngle->kD;
		PIDFeedAngleParams[3] = *m_Config->Instance()->pids->pidFeedAngle->thOut;
		PIDFeedAngleParams[4] = *m_Config->Instance()->pids->pidFeedAngle->thIOut;

		PIDFeedSpeedParams[0] = *m_Config->Instance()->pids->pidFeedSpeed->kP;
		PIDFeedSpeedParams[1] = *m_Config->Instance()->pids->pidFeedSpeed->kI;
		PIDFeedSpeedParams[2] = *m_Config->Instance()->pids->pidFeedSpeed->kD;
		PIDFeedSpeedParams[3] = *m_Config->Instance()->pids->pidFeedSpeed->thOut;
		PIDFeedSpeedParams[4] = *m_Config->Instance()->pids->pidFeedSpeed->thIOut;

		kFeedSemiSpeed = *m_Config->Instance()->control->gun->feedSemiSpeed;
		kFeedBurstSpeed = *m_Config->Instance()->control->gun->feedBurstSpeed;
		kFeedAutoSpeed = *m_Config->Instance()->control->gun->feedAutoSpeed;

		/*m_RCListener->AddOnChange([](const RemoteStatus& value) {
			SPDLOG_TRACE("@RemoteData=[$ch0={},$ch1={},$ch2={},$ch3={},$ch4={},$sw0={},$sw1={}]",
				value.ch[0], value.ch[1], value.ch[2], value.ch[3], value.ch[4], value.sw[0], value.sw[1]); });*/
		/*m_PhototubeListener->AddOnChange([this](const PhototubeStatus& value)
		{
			SPDLOG_TRACE("@Phototube=[$status_pt={}]", value.m_Status);
		});*/
		//如果射击数据（0x0207）有更新，则累加已发射的子弹数
		m_RefereeShootDataListener->AddOnChange([this](const ShootData& value, const ShootData& lastValue)
		{
			++m_CurBulletShotNum;
		});
		m_FlagInitFeed = true;
		m_FeedMode = FeedMode::Disable;

		m_PIDFeedAngle.SetParams(PIDFeedAngleParams);
		m_PIDFeedAngle.SetFlagAngleLoop();
		
		m_PIDFeedSpeed.SetParams(PIDFeedSpeedParams);
		//m_PIDFeedSpeed.SetOutputLimit(-(*m_Config->Instance()->pids->pidFeedSpeed->thOut), 0);
		m_RPMFdbFilter.SetState(0.25, 0.006);

		m_FeedEcdHelper.SetRatio(kSpeedToMotorRPMCoef);
		m_RampEcdSumSet.SetState(30);
	}

	void InitFeed()
	{
		m_FeedEcdHelper.Reset();
		//m_FeedMotorEcdSumSetRamp = m_FeedMotorStatus.m_Encoding[Feed];
		m_FeedMotorEcdSumSet = m_FeedMotorStatus.m_Encoding[Feed] * kMotorEcdToRadCoef;
		m_CurBulletShotNum = 0;

		m_FeedMode = m_LastFeedMode = FeedMode::Disable;
		m_FlagInitFeed = false;
		m_FlagInPosition = true;
	}

	void UpdateFeedSensorFeedback()
	{
		static uint16_t lastHeat = 0;   //上次读取的热量值

		m_FeedMotorStatus = m_MotorFeedListener->Get();
		m_FeedSensorValues.rc = m_RCListener->Get();
		m_FeedSensorValues.keyboardMode = (m_FeedSensorValues.rc.sw[0] == kRCSwUp && m_FeedSensorValues.rc.sw[1] == kRCSwUp);
		//m_FeedSensorValues.gimbalCtrlMode = m_GimbalCtrlTask->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理
		m_FeedSensorValues.feedAngle = m_FeedEcdHelper.CalcEcdSum(m_FeedMotorStatus.m_Encoding[Feed]);
		if (std::fabs(m_FeedMotorEcdSumSet - m_FeedSensorValues.feedAngle) < 0.05)
			m_FlagInPosition = true;

		m_FeedSensorValues.gimbalStatus = m_GimbalStatusListener->Get();
		m_FeedSensorValues.fricStatus = m_FricStatusListener->Get();
		m_FeedSensorValues.refereePowerHeatData = m_RefereePowerHeatDataListener->Get();
		m_FeedSensorValues.refereeRobotStatus = m_RefereeRobotStatusListener->Get();

		//如果当前获取的热量低于历史热量，则将已发射的子弹数清零
		if (m_FeedSensorValues.refereePowerHeatData.m_Shooter17Id1Heat < lastHeat
			&& m_FeedMode == Disable)
		{
			m_CurBulletShotNum = 0;
		}
		lastHeat = m_FeedSensorValues.refereePowerHeatData.m_Shooter17Id1Heat;

		/*SPDLOG_TRACE("@FeedAngle=[$FAngle={}]",
			m_FeedSensorValues.relativeAngle);*/
	}

	void FeedModeSet();

	//发送电流给2006
	void FeedRotateCtrl(bool disable, double expDeltaAngle, int16_t feedSpeed);

	void FeedCtrl();

	auto ExecuteProc() -> void override
	{
		using Clock = std::chrono::high_resolution_clock;
		using TimeStamp = Clock::time_point;

		TimeStamp lastTime = Clock::now();
		while (true)
		{
			while (1000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();

			UpdateFeedSensorFeedback();

			if (m_FlagInitFeed)
				InitFeed();
			
			FeedModeSet();
			FeedCtrl();
		}

	}

private:
	ossian::Utils::ConfigLoader<Config::ConfigSchema>* m_Config;
	ossian::IOData<RemoteStatus>* m_RCListener;  //遥控器
	ossian::IOData<PowerHeatData>* m_RefereePowerHeatDataListener;
	ossian::IOData<RobotStatus>* m_RefereeRobotStatusListener;
	ossian::IOData<ShootData>* m_RefereeShootDataListener;
	ossian::IOData<GunFeedMotorsModel>* m_MotorFeedListener;
	ossian::IOData<GimbalStatus>* m_GimbalStatusListener;
	ossian::IOData<FricStatus>* m_FricStatusListener;

	Gun* m_Gun;
	GunFeedMotorsModel m_FeedMotorStatus;
	EncoderHelper m_FeedEcdHelper;
	RampFunction m_RampEcdSumSet;

	FeedMode m_FeedMode, m_LastFeedMode;

	struct FeedSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		PowerHeatData refereePowerHeatData;
		RobotStatus refereeRobotStatus;
		double feedAngle; 
		GimbalStatus gimbalStatus;
		FricStatus fricStatus;
		//double refereeCurHeat, refereeHeatLimit;
		bool keyboardMode;
	} m_FeedSensorValues;

	bool m_FlagInitFeed, m_FlagInPosition;
	std::chrono::high_resolution_clock::time_point m_LastShootTimestamp;
	std::chrono::high_resolution_clock::time_point m_TimestampInit;
	std::chrono::high_resolution_clock::time_point m_TimestampLastInPosition;
	std::atomic<int> m_CurBulletShotNum{}; //在热量持续上升的过程中，累积打出的子弹数
	PIDController m_PIDFeedAngle, m_PIDFeedSpeed;
	double m_FeedMotorEcdSumSet;  //2006位置设定值
	double m_FeedMotorEcdSumSetRamp;  //2006位置设定值的斜坡值
	FirstOrderFilter m_RPMFdbFilter;
};

#endif // OSSIAN_GUN_HPP