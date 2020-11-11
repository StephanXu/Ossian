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
#include <ctime>

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
	static constexpr int16_t kFricLowSpeed = 100;

	enum MotorPosition
	{
		FricBelow = 0, FricUpper
	};
	
	OSSIAN_SERVICE_SETUP(FricCtrlTask(ossian::IOData<RemoteStatus>* remote,
		ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		Gun* gun,
		ossian::IOData<GunFricMotorsModel>* motorsFricListener,
		ossian::IOData<GimbalStatus>* gimbalStatusListener,
		ossian::IOData<FricStatus>* fricStatusSender))
		: m_RCListener(remote)
		, m_Config(config)
		, m_Gun(gun)
		, m_MotorsFricListener(motorsFricListener)
		, m_GimbalStatusListener(gimbalStatusListener)
		, m_FricStatusSender(fricStatusSender)
	{
		PIDFricSpeedParams[0] = *m_Config->Instance()->pids->pidFricSpeed->kP;
		PIDFricSpeedParams[1] = *m_Config->Instance()->pids->pidFricSpeed->kI;
		PIDFricSpeedParams[2] = *m_Config->Instance()->pids->pidFricSpeed->kD;
		PIDFricSpeedParams[3] = *m_Config->Instance()->pids->pidFricSpeed->thOut;
		PIDFricSpeedParams[4] = *m_Config->Instance()->pids->pidFricSpeed->thIOut;

		kFricSpeed12 = *m_Config->Instance()->control->gun->fricSpeed12;
		kFricSpeed15 = *m_Config->Instance()->control->gun->fricSpeed15;
		kFricSpeed18 = *m_Config->Instance()->control->gun->fricSpeed18;
		kFricSpeed30 = *m_Config->Instance()->control->gun->fricSpeed30;

		m_FlagInitFric = true;
		m_FricMode = FricMode::Disable;

		PIDController pidFricSpeed;
		pidFricSpeed.SetParams(PIDFricSpeedParams);
		m_PIDFricSpeed.fill(pidFricSpeed);

		FirstOrderFilter rpmFdbFilter(0.25, 0.003);
		m_RPMFdbFilters.fill(rpmFdbFilter);
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
		m_FricSensorValues.gimbalStatus = m_GimbalStatusListener->Get();
		//m_FricSensorValues.gimbalCtrlMode = m_GimbalCtrlTask->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理
	}

	/*bool Stopped() 
	{ 
		return (m_FricMode == FricMode::Disable
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
			//SPDLOG_INFO("@FricInterval=[$timefric={}]", std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count());
			while (1000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
			}
			lastTime = Clock::now();

			m_FricMotorsStatus = m_MotorsFricListener->Get();

			UpdateFricSensorFeedback();
			if (m_FlagInitFric)
				InitFric();

			FricModeSet();
			m_Status.m_Mode = m_FricMode;
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

	FricStatus m_Status;
	ossian::IOData<FricStatus>* m_FricStatusSender;

	GunFricMotorsModel m_FricMotorsStatus;
	Gun* m_Gun;

	struct FricSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		GimbalStatus gimbalStatus;
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
		Disable, Init, Stop, Reload, Reverse, Semi, Burst, Auto
	};

	static constexpr double kMotorEcdToRadCoef = 2 * M_PI / 8192.0 / 36.0;  //电机编码值---拨盘旋转角度
	static constexpr int kNumCells = 8;
	static constexpr double kAnglePerCell = 2 * M_PI / kNumCells;
	static constexpr std::array<uint16_t, kNumCells> kFeedMidEcds{ 100,500,1000,2000,3000,4000,5000,6000 };

	//遥控器解析
	static constexpr int16_t kGunRCDeadband = 10; //拨轮死区
	static constexpr size_t kShootModeChannel = 1; //4

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
	static std::array<double, 5> PIDFeedAngleParams;
	static std::array<double, 5> PIDFeedSpeedParams;

	OSSIAN_SERVICE_SETUP(FeedCtrlTask(ossian::IOData<RemoteStatus>* remote,
		ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		Gun* gun,
		ossian::IOData<GunFeedMotorsModel>* motorFeedListener,
		ossian::IOData<PowerHeatData>* powerHeatDataListener,
		ossian::IOData<RobotStatus>* robotStatusListener,
		ossian::IOData<ShootData>* shootDataListener,
		ossian::IOData<PhototubeStatus>* phototubeListener,
		ossian::IOData<GimbalStatus>* gimbalStatusListener,
		ossian::IOData<FricStatus>* fricStatusListener))
		: m_RCListener(remote)
		, m_Config(config)
		, m_Gun(gun)
		, m_MotorFeedListener(motorFeedListener)
		, m_RefereePowerHeatDataListener(powerHeatDataListener)
		, m_RefereeRobotStatusListener(robotStatusListener)
		, m_RefereeShootDataListener(shootDataListener)
		, m_PhototubeListener(phototubeListener)
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

		/*kFeedNormalSpeed = *m_Config->Instance()->control->gun->kfeednormalspeed();
		kFeedSemiSpeed = *m_Config->Instance()->control->gun->kfeedsemispeed();
		kFeedBurstSpeed = *m_Config->Instance()->control->gun->kfeedburstspeed();
		kFeedAutoSpeed = *m_Config->Instance()->control->gun->kfeedautospeed();*/

		/*m_RCListener->AddOnChange([](const RemoteStatus& value) {
			SPDLOG_INFO("@RemoteData=[$ch0={},$ch1={},$ch2={},$ch3={},$ch4={},$sw0={},$sw1={}]",
				value.ch[0], value.ch[1], value.ch[2], value.ch[3], value.ch[4], value.sw[0], value.sw[1]); });*/
		/*m_PhototubeListener->AddOnChange([this](const PhototubeStatus& value)
		{
			SPDLOG_INFO("@Phototube=[$status_pt={}]", value.m_Status);
		});*/
		//如果射击数据（0x0207）有更新，则累加已发射的子弹数
		/*m_RefereeShootDataListener->AddOnChange([this](const ShootData& value)
		{
			++m_CurBulletShotNum;
		});*/
		m_FlagInitFeed = true;
		m_FlagFindEcdMid = true;
		m_FeedMode = FeedMode::Disable;

		m_PIDFeedAngle.SetParams(PIDFeedAngleParams);
		m_PIDFeedAngle.SetFlagAngleLoop();
		
		m_PIDFeedSpeed.SetParams(PIDFeedSpeedParams);
		m_RPMFdbFilter.SetState(0.25, 0.006);
	}

	void FindEcdMid()
	{
		double absMinAngle = DBL_MAX;
		for (const auto& ecdMid : kFeedMidEcds)
		{
			double relativeAngle = RelativeEcdToRad(m_FeedMotorStatus.m_Encoding[FeedCtrlTask::Feed],
				ecdMid) / kSpeedToMotorRPMCoef;
			if (relativeAngle > 0 && fabs(relativeAngle) < absMinAngle)  //拨弹轮只能正转
			{
				absMinAngle = fabs(relativeAngle);
				m_FeedEcdMid = ecdMid;
			}
		}
		m_FlagFindEcdMid = false;
	}

	void InitFeed()
	{
		static bool firstClosing = true;
		if (fabs(m_FeedSensorValues.relativeAngle) < 0.01)
		{
			if (firstClosing)
			{
				m_TimestampInit = std::chrono::high_resolution_clock::now();
				firstClosing = false;
			}

			long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - m_TimestampInit).count();
			//std::cerr << interval << std::endl;
			if (interval > 2000 && m_TimestampInit != std::chrono::high_resolution_clock::time_point())  //在中值处稳定一段时间
			{
				std::cerr << "Feed Init Done!" << std::endl;
				m_FeedMode = FeedMode::Disable;
				m_PIDFeedSpeed.Reset();
				m_CurBulletShotNum = 0;
				m_LastShootTimestamp = std::chrono::high_resolution_clock::time_point();
				m_RPMFdbFilter.Reset();
				m_PIDFeedAngle.Reset();
				m_FlagInPosition = false;

				firstClosing = true;
				m_TimestampInit = std::chrono::high_resolution_clock::time_point();

				m_FlagInitFeed = false;
			}
			else
			{
				m_FeedMode = FeedMode::Init;
				if (m_FeedSensorValues.rc.sw[kShootModeChannel] != kRCSwMid)  //右侧开关保持居中，云台归中，否则失能
					m_FeedMode = FeedMode::Disable;

				return;
			}

			//m_AngleSet = m_FeedSensorValues.relativeAngle;
			//m_EcdMid = m_FeedMotorStatus.m_Encoding[FeedCtrlTask::Feed];

		}
		else
		{
			m_FeedMode = FeedMode::Init;
			if (m_FeedSensorValues.rc.sw[kShootModeChannel] != kRCSwMid)  //右侧开关保持居中，云台归中，否则失能
				m_FeedMode = FeedMode::Disable;

			return;
		}
	}

	void UpdateFeedSensorFeedback()
	{
		static uint16_t lastHeat = 0;      //上次读取的热量值

		m_FeedSensorValues.rc = m_RCListener->Get();
		//m_FeedSensorValues.gimbalCtrlMode = m_GimbalCtrlTask->GimbalCtrlSrc();  //[TODO] 增加对自瞄模式射击的处理
		m_FeedSensorValues.phototubeStatus = m_PhototubeListener->Get();
		
		m_FeedSensorValues.relativeAngle = 
			RelativeEcdToRad(m_FeedMotorStatus.m_Encoding[FeedCtrlTask::Feed], m_FeedEcdMid) / kSpeedToMotorRPMCoef;
		m_FeedSensorValues.gimbalStatus = m_GimbalStatusListener->Get();
		m_FeedSensorValues.fricStatus = m_FricStatusListener->Get();
		/*m_FeedSensorValues.refereePowerHeatData = m_RefereePowerHeatDataListener->Get();
		m_FeedSensorValues.refereeRobotStatus = m_RefereeRobotStatusListener->Get();*/

		//如果当前获取的热量低于历史热量，则将已发射的子弹数清零
		if (m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat < lastHeat)
			m_CurBulletShotNum = 0;
		lastHeat = m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat;
	}

	void FeedModeSet();

	//发送电流给2006
	void FeedRotateCtrl(bool disable=true, double deltaAngle=0);

	//void AutoReloadCtrl();

	//void SingleShotCtrl(int speedSet);

	void FeedCtrl();

	auto ExecuteProc() -> void override
	{
		using Clock = std::chrono::high_resolution_clock;
		using TimeStamp = Clock::time_point;

		TimeStamp lastTime = Clock::now();
		//volatile clock_t lastTime = clock();
		while (true)
		{
			//auto interval = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count();
			//double interval = (double)(clock() - lastTime) / CLOCKS_PER_SEC * 1000.0;
			//SPDLOG_INFO("@FeedInterval=[$timefeed={}]", interval);
			while (1000 > std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastTime).count())
			{
				std::this_thread::yield();
				//interval = (double)(clock() - lastTime) / CLOCKS_PER_SEC * 1000.0;
			}
			
			//lastTime = clock();
			lastTime = Clock::now();
			//SPDLOG_INFO("@FeedSpeedSet=[$normal={},$semi={},$burst={},$auto={}]", kFeedNormalSpeed, kFeedSemiSpeed, kFeedBurstSpeed, kFeedAutoSpeed);

			m_FeedMotorStatus = m_MotorFeedListener->Get();

			//找离初始位置最近的击发中值
			if (m_FlagFindEcdMid)
				FindEcdMid();

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
	ossian::IOData<PhototubeStatus>* m_PhototubeListener;
	ossian::IOData<GimbalStatus>* m_GimbalStatusListener;
	ossian::IOData<FricStatus>* m_FricStatusListener;

	Gun* m_Gun;
	GunFeedMotorsModel m_FeedMotorStatus;
	
	FeedMode m_FeedMode;
	struct FeedSensorFeedback
	{
		RemoteStatus rc;	 //遥控器数据
		PowerHeatData refereePowerHeatData;
		RobotStatus refereeRobotStatus;
		PhototubeStatus phototubeStatus;
		double relativeAngle;  //拨盘角度	
		GimbalStatus gimbalStatus;
		FricStatus fricStatus;
		//double refereeCurHeat, refereeHeatLimit;
	} m_FeedSensorValues;

	bool m_FlagInitFeed, m_FlagFindEcdMid;
	uint16_t m_FeedEcdMid=0;
	std::chrono::high_resolution_clock::time_point m_LastShootTimestamp;
	std::chrono::high_resolution_clock::time_point m_TimestampInit;
	std::atomic<int> m_CurBulletShotNum; //在热量持续上升的过程中，累积打出的子弹数
	PIDController m_PIDFeedAngle, m_PIDFeedSpeed;
	bool m_FlagInPosition;
	FirstOrderFilter m_RPMFdbFilter;
};

#endif // OSSIAN_GUN_HPP