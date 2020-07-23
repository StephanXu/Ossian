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
#include "Phototube.hpp"
#include <chrono>
#include <array>
#include <atomic>

//constexpr size_t kNumGunFricMotorsFake = 3;
constexpr size_t kNumGunFricMotors = 2; 
constexpr size_t kNumGunFeedMotors = 1;

OSSIAN_MULTIPLE_MOTORS_STATUS(GunFricMotorsModel, kNumGunFricMotors);
OSSIAN_MULTIPLE_MOTORS_STATUS(GunFeedMotorsModel, kNumGunFeedMotors);

using hrClock = std::chrono::high_resolution_clock;
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

	hrClock::time_point m_LastRefresh;
	
};

class FricCtrlTask : public ossian::IExecutable
{
public:
	//摩擦轮转速期望rpm [TODO]实验得出不同等级下的射击初速度上限所对应的摩擦轮转速期望
	static int16_t kFricSpeed12;
	static int16_t kFricSpeed15;
	static int16_t kFricSpeed18;
	static int16_t kFricSpeed30;

	//遥控器解析
	static constexpr int16_t kGunRCDeadband = 50; //拨轮死区
	static constexpr size_t kShootModeChannel = 1;

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	//pid参数
	static std::array<double, 5> PIDFricSpeedParams;

	//摩擦轮转速下限
	static constexpr int16_t kFricLowSpeed = 10;

	enum MotorPosition
	{
		FricBelow = 0, FricUpper
	};

	enum FricMode
	{
		Disable, Enable
	};
	
	OSSIAN_SERVICE_SETUP(FricCtrlTask(ossian::IOData<RemoteStatus>* remote,
		GimbalCtrlTask* gimbalCtrlTask,
		Utils::ConfigLoader* config,
		Gun* gun,
		ossian::IOData<GunFricMotorsModel>* motorsFricListener))
		: m_RCListener(remote)
		, m_GimbalCtrlTask(gimbalCtrlTask)
		, m_Config(config)
		, m_Gun(gun)
		, m_MotorsFricListener(motorsFricListener)
	{
		using OssianConfig::Configuration;
		PIDFricSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->kp();
		PIDFricSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->ki();
		PIDFricSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->kd();
		PIDFricSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->thout();
		PIDFricSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->thiout();

		kFricSpeed12 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed12();
		kFricSpeed15 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed15();
		kFricSpeed18 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed18();
		kFricSpeed30 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed30();

		m_FlagInitFric = true;

		PIDController pidFricSpeed;
		pidFricSpeed.SetParams(PIDFricSpeedParams);
		m_PIDFricSpeed.fill(pidFricSpeed);

		FirstOrderFilter rpmFdbFilter(0.25, 0.003);
		m_RPMFdbFilters.fill(rpmFdbFilter);

		m_RCListener->AddOnChange([](const RemoteStatus& value) {
			SPDLOG_INFO("@RemoteData=[$ch0={},$ch1={},$ch2={},$ch3={},$ch4={}]",
				value.ch[0], value.ch[1], value.ch[2], value.ch[3], value.ch[4]); });
	}

	void InitFric()
	{
		m_FricMode = FricMode::Disable;

		std::for_each(m_PIDFricSpeed.begin(), m_PIDFricSpeed.end(), [](PIDController& x) {x.Reset(); });
		std::for_each(m_RPMFdbFilters.begin(), m_RPMFdbFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });

		m_FlagInitFric = false;
	}

	void UpdateFricSensorFeedback()
	{
		m_FricSensorValues.rc = m_RCListener->Get();
		m_FricSensorValues.gimbalInputSrc = m_GimbalCtrlTask->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理
	}

	bool Stopped() 
	{ 
		return (m_FricMode == FricMode::Disable
			|| std::abs(m_FricMotorsStatus.m_RPM[FricBelow]) < kFricLowSpeed
			|| std::abs(m_FricMotorsStatus.m_RPM[FricUpper]) < kFricLowSpeed);
	}

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
			while (3000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();

			m_FricMotorsStatus = m_MotorsFricListener->Get();

			UpdateFricSensorFeedback();
			if (m_FlagInitFric)
				InitFric();

			FricModeSet();
			FricExpSpeedSet();
			FricCtrl();
		}
	}


private:
	Utils::ConfigLoader* m_Config;
	ossian::IOData<RemoteStatus>* m_RCListener;  //遥控器
	GimbalCtrlTask* m_GimbalCtrlTask;

	ossian::IOData<GunFricMotorsModel>* m_MotorsFricListener;
	
	GunFricMotorsModel m_FricMotorsStatus;
	Gun* m_Gun;

	struct FricSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		GimbalCtrlTask::GimbalInputSrc gimbalInputSrc;
		//double refereeCurHeat, refereeHeatLimit;
		uint8_t shooter_heat0_speed_limit = 30;

	} m_FricSensorValues;

	bool m_FlagInitFric;
	FricMode m_FricMode;

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
		Stop, Reload, Reverse, Semi, Burst, Auto
	};

	//遥控器解析
	static constexpr int16_t kGunRCDeadband = 10; //拨轮死区
	static constexpr size_t kShootModeChannel = 4; //4

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	//枪口热量机制
	static constexpr int kHeatPerBullet = 10;

	//拨弹轮
	static constexpr double kSpeedToMotorRPMCoef = 36;
	static constexpr int kBurstBulletNum = 3; //点射的子弹发射数量
	static constexpr int16_t kFeedJamRPM = 5;     //判断卡弹的拨弹轮转速阈值

	static int16_t kFeedNormalSpeed; //自动补弹上膛或反转时，拨弹轮供弹的转速
	static int16_t kFeedSemiSpeed;   //单发时，拨弹轮供弹的转速
	static int16_t kFeedBurstSpeed;  //点射时，拨弹轮供弹的转速
	static int16_t kFeedAutoSpeed;   //连发时，拨弹轮供弹的转速   点射>连发>单发

	//pid参数
	static std::array<double, 5> PIDFeedSpeedParams;

	OSSIAN_SERVICE_SETUP(FeedCtrlTask(ossian::IOData<RemoteStatus>* remote,
		GimbalCtrlTask* gimbalCtrlTask,
		FricCtrlTask* fricCtrlTask,
		Utils::ConfigLoader* config,
		Gun* gun,
		ossian::IOData<GunFeedMotorsModel>* motorFeedListener,
		ossian::IOData<PowerHeatData>* powerHeatDataListener,
		ossian::IOData<RobotStatus>* robotStatusListener,
		ossian::IOData<ShootData>* shootDataListener,
		ossian::IOData<PhototubeStatus>* phototubeListener))
		: m_RCListener(remote)
		, m_GimbalCtrlTask(gimbalCtrlTask)
		, m_FricCtrlTask(fricCtrlTask)
		, m_Config(config)
		, m_Gun(gun)
		, m_MotorFeedListener(motorFeedListener)
		, m_RefereePowerHeatDataListener(powerHeatDataListener)
		, m_RefereeRobotStatusListener(robotStatusListener)
		, m_RefereeShootDataListener(shootDataListener)
		, m_PhototubeListener(phototubeListener)
	{
		using OssianConfig::Configuration;

		PIDFeedSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->kp();
		PIDFeedSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->ki();
		PIDFeedSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->kd();
		PIDFeedSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->thout();
		PIDFeedSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->thiout();

		kFeedNormalSpeed = m_Config->Instance<Configuration>()->mutable_gun()->kfeednormalspeed();
		kFeedSemiSpeed = m_Config->Instance<Configuration>()->mutable_gun()->kfeedsemispeed();
		kFeedBurstSpeed = m_Config->Instance<Configuration>()->mutable_gun()->kfeedburstspeed();
		kFeedAutoSpeed = m_Config->Instance<Configuration>()->mutable_gun()->kfeedautospeed();

		/*m_PhototubeListener->AddOnChange([this](const PhototubeStatus& value)
		{
			SPDLOG_INFO("@Phototube=[$status_pt={}]", value.m_Status);
		});*/
		//如果射击数据（0x0207）有更新，则累加已发射的子弹数
		m_RefereeShootDataListener->AddOnChange([this](const ShootData& value)
		{
			++m_CurBulletShotNum;
		});
		m_FlagInitFeed = true;

		m_PIDFeedSpeed.SetParams(PIDFeedSpeedParams);
		m_RPMFdbFilter.SetState(0.25, 0.003);
	}

	void InitFeed()
	{
		m_FeedMode = FeedMode::Stop;
		m_PIDFeedSpeed.Reset();
		m_CurBulletShotNum = 0;
		m_LastShootTimestamp = hrClock::time_point();
		m_RPMFdbFilter.Reset();

		m_FlagInitFeed = false;
	}

	void UpdateFeedSensorFeedback()
	{
		static uint16_t lastHeat = 0;      //上次读取的热量值

		m_FeedSensorValues.rc = m_RCListener->Get();
		m_FeedSensorValues.gimbalInputSrc = m_GimbalCtrlTask->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理
		m_FeedSensorValues.phototubeStatus = m_PhototubeListener->Get();

		m_FeedSensorValues.refereePowerHeatData = m_RefereePowerHeatDataListener->Get();
		m_FeedSensorValues.refereeRobotStatus = m_RefereeRobotStatusListener->Get();

		//如果当前获取的热量低于历史热量，则将已发射的子弹数清零
		if (m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat < lastHeat)
			m_CurBulletShotNum = 0;
		lastHeat = m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat;
	}

	void FeedModeSet();

	//发送电流给2006
	void FeedRotateCtrl(bool stop = false, int speedSet = 0, bool reverse = false);

	void AutoReloadCtrl();

	void SingleShotCtrl(int speedSet);

	void FeedCtrl();

	auto ExecuteProc() -> void override
	{
		using Clock = std::chrono::high_resolution_clock;
		using TimeStamp = Clock::time_point;

		TimeStamp lastTime = Clock::now();
		while (true)
		{
			while (3000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();

			m_FeedMotorStatus = m_MotorFeedListener->Get();

			UpdateFeedSensorFeedback();
			if (m_FlagInitFeed)
				InitFeed();

			FeedModeSet();
			FeedCtrl();
		}

	}

private:
	Utils::ConfigLoader* m_Config;
	ossian::IOData<RemoteStatus>* m_RCListener;  //遥控器
	ossian::IOData<PowerHeatData>* m_RefereePowerHeatDataListener;
	ossian::IOData<RobotStatus>* m_RefereeRobotStatusListener;
	ossian::IOData<ShootData>* m_RefereeShootDataListener;
	ossian::IOData<GunFeedMotorsModel>* m_MotorFeedListener;
	ossian::IOData<PhototubeStatus>* m_PhototubeListener;

	GimbalCtrlTask* m_GimbalCtrlTask;
	FricCtrlTask* m_FricCtrlTask;
	Gun* m_Gun;
	GunFeedMotorsModel m_FeedMotorStatus;
	
	FeedMode m_FeedMode;
	struct FeedSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		GimbalCtrlTask::GimbalInputSrc gimbalInputSrc;
		PowerHeatData refereePowerHeatData;
		RobotStatus refereeRobotStatus;
		PhototubeStatus phototubeStatus;
		//double refereeCurHeat, refereeHeatLimit;
	} m_FeedSensorValues;

	bool m_FlagInitFeed;
	hrClock::time_point m_LastShootTimestamp;
	std::atomic<int> m_CurBulletShotNum; //在热量持续上升的过程中，累积打出的子弹数
	PIDController m_PIDFeedSpeed;
	FirstOrderFilter m_RPMFdbFilter;
};

#endif // OSSIAN_GUN_HPP