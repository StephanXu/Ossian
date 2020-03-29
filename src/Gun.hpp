#ifndef OSSIAN_GUN_HPP
#define OSSIAN_GUN_HPP

#include <ossian/Motor.hpp>
#include <ossian/IOData.hpp>

#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gimbal.hpp"
#include "Referee.hpp"

#include <chrono>
#include <array>
#include <atomic>

class Gun
{
public:

	//摩擦轮转速期望rpm [TODO]实验得出不同等级下的射击初速度上限所对应的摩擦轮转速期望
	static int16_t kFricSpeed12;
	static int16_t kFricSpeed15;
	static int16_t kFricSpeed18;
	static int16_t kFricSpeed30;

	//遥控器解析
	static constexpr int16_t kGunRCDeadband = 50; //拨轮死区
	static constexpr size_t kShootModeChannel = 4;

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	//枪口热量机制
	static constexpr int kHeatPerBullet = 10;

	//拨弹轮
	static constexpr int kBurstBulletNum = 3; //点射的子弹发射数量
	static constexpr int16_t kFeedJamRPM = 5;     //判断卡弹的拨弹轮转速阈值

	static int16_t kFeedNormalRPM; //自动补弹上膛或反转时，拨弹轮供弹的转速
	static int16_t kFeedSemiRPM;   //单发时，拨弹轮供弹的转速
	static int16_t kFeedBurstRPM;  //点射时，拨弹轮供弹的转速
	static int16_t kFeedAutoRPM;   //连发时，拨弹轮供弹的转速   点射>连发>单发
	
	//pid参数
	static std::array<double, 5> PIDFricSpeedParams;
	static std::array<double, 5> PIDFeedSpeedParams;

	enum MotorPosition
	{
		FricBelow, FricUpper, Feed
	};

	enum FricMode
	{
		Disable, Enable
	};

	enum FeedMode
	{
		Stop, Reload, Reverse, Semi, Burst, Auto
	};

	OSSIAN_SERVICE_SETUP(Gun(ossian::MotorManager* motorManager, 
							 ossian::IOData<RemoteStatus>* remote, 
							 Gimbal* gimbal, 
							 Utils::ConfigLoader* config, 
							 ossian::IOData<PowerHeatData>* powerHeatDataListener,
							 ossian::IOData<RobotStatus>* robotStatusListener,
							 ossian::IOData<ShootData>* shootDataListener))
		: m_MotorManager(motorManager)
		, m_RC(remote)
		, m_Gimbal(gimbal)
		, m_Config(config)
		, m_RefereePowerHeatDataListener(powerHeatDataListener)
		, m_RefereeRobotStatusListener(robotStatusListener)
		, m_RefereeShootDataListener(shootDataListener)
	{
		using OssianConfig::Configuration;
		PIDFricSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->kp();
		PIDFricSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->ki();
		PIDFricSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->kd();
		PIDFricSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->thout();
		PIDFricSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->thiout();

		PIDFeedSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->kp();
		PIDFeedSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->ki();
		PIDFeedSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->kd();
		PIDFeedSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->thout();
		PIDFeedSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->thiout();

		kFricSpeed12 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed12();
		kFricSpeed15 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed15();
		kFricSpeed18 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed18();
		kFricSpeed30 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed30();

		kFeedNormalRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeednormalrpm();
		kFeedSemiRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeedsemirpm();
		kFeedBurstRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeedburstrpm();
		kFeedAutoRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeedautorpm();

		//如果射击数据（0x0207）有更新，则累加已发射的子弹数
		m_RefereeShootDataListener->AddOnChange([this](const ShootData& value)
		{
			++m_CurBulletShotNum;
		});

		m_FlagInitFric = m_FlagInitFeed = true;

		PIDController pidFricSpeed;
		pidFricSpeed.SetParams(PIDFricSpeedParams);
		m_PIDFricSpeed.fill(pidFricSpeed);

		m_PIDFeedSpeed.SetParams(PIDFeedSpeedParams);

	}

	void InitFric()
	{
		m_FricMode = FricMode::Disable;
		m_FricMotorMsgCheck.fill(false);

		std::for_each(m_PIDFricSpeed.begin(), m_PIDFricSpeed.end(), [](PIDController& x) {x.Reset(); });

		m_FlagInitFric = false;
	}

	void InitFeed()
	{
		m_FeedMode = FeedMode::Stop;
		m_PIDFeedSpeed.Reset();
		m_CurBulletShotNum = 0;

		m_FlagInitFeed = false;
	}

	auto AddMotor(MotorPosition position,
		const std::string location,
		const unsigned int motorId,
		const unsigned int writerCanId)->void
	{
		if (position == FricBelow || position == FricUpper)
			m_Motors[position] =
			m_MotorManager->AddMotor<ossian::DJIMotor>(
				location,
				m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
				[this, position](const std::shared_ptr<ossian::DJIMotor>& motor)
				{
					MotorFricReceiveProc(motor, position);
				},
				motorId);
		else if (position == Feed)
			m_Motors[position] =
				m_MotorManager->AddMotor<ossian::DJIMotor>(
					location,
					m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
					[this, position](const std::shared_ptr<ossian::DJIMotor>& motor)
					{
						MotorFeedReceiveProc(motor, position);
					},
					motorId);
	}

	void UpdateGunSensorFeedback()
	{
		static uint16_t lastHeat = 0;      //上次读取的热量值

		m_GunSensorValues.rc = m_RC->Get();
		m_GunSensorValues.gimbalInputSrc = m_Gimbal->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理

		m_GunSensorValues.refereePowerHeatData = m_RefereePowerHeatDataListener->Get();
		m_GunSensorValues.refereeRobotStatus = m_RefereeRobotStatusListener->Get();

		//如果当前获取的热量低于历史热量，则将已发射的子弹数清零
		if (m_GunSensorValues.refereePowerHeatData.m_Shooter17Heat < lastHeat)
			m_CurBulletShotNum = 0;
		lastHeat = m_GunSensorValues.refereePowerHeatData.m_Shooter17Heat;		
	}

	bool MicroSwitchStatus() { return false; } //获取枪口微动开关 or 光电对管的状态

	void FricModeSet();

	void FricExpSpeedSet();

	void FricCtrl();

	void FeedModeSet();

	//发送电流给2006
	void FeedRotateCtrl(bool stop=false, int rpmSet=0, bool reverse=false);

	void AutoReloadCtrl();

	void SingleShotCtrl(int rpmSet);

	void FeedCtrl();

	auto MotorFricReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor, MotorPosition position)->void
	{
		m_FricMotorMsgCheck[position] = true;
		if (!(m_FricMotorMsgCheck[FricBelow] && m_FricMotorMsgCheck[FricUpper]))
			return;

		UpdateGunSensorFeedback();
		if (m_FlagInitFric)
			InitFric();

		FricModeSet();
		FricExpSpeedSet();
		FricCtrl();
		m_FricMotorMsgCheck.fill(false);
	}

	auto MotorFeedReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor, MotorPosition position)->void
	{
		UpdateGunSensorFeedback();
		if (m_FlagInitFeed)
			InitFeed();

		FeedModeSet();
		FeedCtrl();
	}


private:
	ossian::MotorManager* m_MotorManager;
	std::array<std::shared_ptr<ossian::DJIMotor>, 3> m_Motors;
	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	Utils::ConfigLoader* m_Config;
	ossian::IOData<RemoteStatus>* m_RC;  //遥控器
	Gimbal* m_Gimbal;
	ossian::IOData<PowerHeatData>* m_RefereePowerHeatDataListener;
	ossian::IOData<RobotStatus>* m_RefereeRobotStatusListener;
	ossian::IOData<ShootData>* m_RefereeShootDataListener;

	struct GunSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		Gimbal::GimbalInputSrc gimbalInputSrc;
		PowerHeatData refereePowerHeatData;
		RobotStatus refereeRobotStatus;
		//double refereeCurHeat, refereeHeatLimit;
		uint8_t shooter_heat0_speed_limit=30;
		
	} m_GunSensorValues;
	
	bool m_FlagInitFric, m_FlagInitFeed;
	std::array<bool, 2> m_FricMotorMsgCheck;
	FricMode m_FricMode;
	FeedMode m_FeedMode;

	std::atomic<int> m_CurBulletShotNum; //在热量持续上升的过程中，累积打出的子弹数
	int16_t m_FricSpeedSet;
	std::array<PIDController, 2> m_PIDFricSpeed;
	PIDController m_PIDFeedSpeed;
};

#endif // OSSIAN_GUN_HPP