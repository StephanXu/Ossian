
#include "Gun.hpp"

int16_t FricCtrlTask::kFricSpeed12 = 0;
int16_t FricCtrlTask::kFricSpeed15 = 0;
int16_t FricCtrlTask::kFricSpeed18 = 0;
int16_t FricCtrlTask::kFricSpeed30 = 0;

int16_t FeedCtrlTask::kFeedNormalRPM = 0;
int16_t FeedCtrlTask::kFeedSemiRPM = 0;
int16_t FeedCtrlTask::kFeedBurstRPM = 0;
int16_t FeedCtrlTask::kFeedAutoRPM = 0;

std::array<double, 5> FricCtrlTask::PIDFricSpeedParams;
std::array<double, 5> FeedCtrlTask::PIDFeedSpeedParams;

void FricCtrlTask::FricModeSet()
{
	static uint8_t lastSw = kRCSwUp;

	//遥控器右侧开关上拨一次，开启摩擦轮；再上拨一次，关闭摩擦轮
	if (m_FricSensorValues.rc.sw[kShootModeChannel] == kRCSwUp && lastSw != kRCSwUp)
	{
		switch (m_FricMode)
		{
		case FricMode::Disable:
			m_FricMode = FricMode::Enable; break;
		case FricMode::Enable:
			m_FricMode = FricMode::Disable; break;
		default:
			m_FricMode = FricMode::Disable; break;
		}
	}
	//如果云台失能，则摩擦轮也失能
	/*else if (m_FricSensorValues.gimbalInputSrc == GimbalCtrlTask::GimbalInputSrc::Disable)
		m_FricMode = FricMode::Disable;*/

	lastSw = m_FricSensorValues.rc.sw[kShootModeChannel];
}

//[TODO] 读取场地加成RFID状态，叠加射击速度加成
void FricCtrlTask::FricExpSpeedSet()
{
	if (m_FricMode == FricMode::Disable)
		m_FricSpeedSet = 0;
	else if (m_FricMode == FricMode::Enable)
	{
		switch (m_FricSensorValues.shooter_heat0_speed_limit)
		{
		case 12:
			m_FricSpeedSet = kFricSpeed12; break;
		case 15:
			m_FricSpeedSet = kFricSpeed15; break;
		case 18:
			m_FricSpeedSet = kFricSpeed18; break;
		case 30:
			m_FricSpeedSet = kFricSpeed30; break;
		default:
			m_FricSpeedSet = 0; break;
		}
	}
}

void FricCtrlTask::FricCtrl()
{
	static std::array<double, kNumGunFricMotors> currentSend{};
	if (m_FricMode == FricMode::Disable)
		currentSend.fill(0);
	else
	{
		currentSend[FricBelow] = m_RPMFdbFilters[FricBelow].Calc(m_FricMotorsStatus.m_RPM[FricBelow]);
		currentSend[FricBelow] = m_PIDFricSpeed[FricBelow].Calc(-m_FricSpeedSet, currentSend[FricBelow]);

		currentSend[FricUpper] = m_RPMFdbFilters[FricUpper].Calc(m_FricMotorsStatus.m_RPM[FricUpper]);
		currentSend[FricUpper] = m_PIDFricSpeed[FricUpper].Calc(m_FricSpeedSet, m_FricMotorsStatus.m_RPM[FricUpper]);
	}
	SPDLOG_INFO("@PIDFricBelow=[$setFB={},$getFB={},$pidoutFB={}]",
		-m_FricSpeedSet,
		m_FricMotorsStatus.m_RPM[FricBelow],
		currentSend[FricBelow]);
	SPDLOG_INFO("@PIDFricUpper=[$setFU={},$getFU={},$pidoutFU={}]",
		m_FricSpeedSet,
		m_FricMotorsStatus.m_RPM[FricUpper],
		currentSend[FricUpper]);
	m_Gun->SendCurrentToMotorsFric(currentSend);
}

//[TODO] 鼠标键盘射击逻辑
void FeedCtrlTask::FeedModeSet()
{
	//若超热量则拨弹轮停转
	bool overheat = m_CurBulletShotNum.load() >= (m_FeedSensorValues.refereeRobotStatus.m_Shooter17HeatLimit
		- m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat) / kHeatPerBullet;
	
	//若摩擦轮停转，则拨弹轮停转
	if (/*overheat || */m_FricCtrlTask->Stopped())
		m_FeedMode = FeedMode::Stop;
	else
	{
		//在打开摩擦轮的情况下：左上角的波轮，向下 单发，向上 连发
		int16_t thumbWheelValue = DeadbandLimit(m_FeedSensorValues.rc.ch[kShootModeChannel], kGunRCDeadband);
		if (thumbWheelValue < 0)
			m_FeedMode = FeedMode::Semi;
		else if (thumbWheelValue == 0)
			m_FeedMode = FeedMode::Reload;
		else
			m_FeedMode = FeedMode::Auto;
	}
	
	//若卡弹则拨弹轮反转
	/*bool jammed = m_FeedMode != FeedMode::Stop && m_FeedMotorStatus.m_RPM[Feed] < kFeedJamRPM;
	if (jammed)
		m_FeedMode = FeedMode::Reverse; */
}

void FeedCtrlTask::FeedRotateCtrl(bool stop, int rpmSet, bool reverse)
{
	double current = 0;
	double get = m_RPMFdbFilter.Calc(m_FeedMotorStatus.m_RPM[FeedCtrlTask::Feed]);
	if (stop)
		current = 0;
	else
	{
		current = m_PIDFeedSpeed.Calc(rpmSet, get);
		if (reverse)
			current *= -1;
	}
	SPDLOG_INFO("@PIDFeed=[$setFd={},$getFd={},$pidoutFd={}]",
		rpmSet,
		get,
		current);
	m_Gun->SendCurrentToMotorFeed(current);
}

void FeedCtrlTask::AutoReloadCtrl()
{
	//如果光电管处无弹，则控制拨弹轮补弹
	if (m_FeedSensorValues.phototubeStatus.m_Status == PhototubeStatus::NO_BULLET)
		FeedRotateCtrl(false, kFeedNormalRPM);
	else //否则，拨弹轮停止
		FeedRotateCtrl(true);
}

/**
 * 由于自动补弹的作用，进入此函数时，微动开关处有弹
 * 拨弹轮转动--->微动开关检测到弹丸离开--->拨弹轮转动送弹--->微动开关检测有弹丸--->拨弹轮停
 */
void FeedCtrlTask::SingleShotCtrl(int rpmSet)
{
	//如果光电管处有弹，则控制拨弹轮送弹
	if (m_FeedSensorValues.phototubeStatus.m_Status == PhototubeStatus::HAS_BULLET)
		FeedRotateCtrl(false, rpmSet);
	else //否则，控制拨弹轮补弹
		AutoReloadCtrl();
}

void FeedCtrlTask::FeedCtrl()
{
	static int shootCnt = 0;
	long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
						 hrClock::now() - m_LastShootTimestamp).count();
	
	if (m_FeedMode == FeedMode::Stop)
		FeedRotateCtrl(true);
	else if (m_FeedMode == FeedMode::Reload)
		AutoReloadCtrl();
	else if (m_FeedMode == FeedMode::Reverse)
		FeedRotateCtrl(false, kFeedNormalRPM, true);
	else if (m_FeedMode == FeedMode::Semi)
	{
		if (interval >= 1000 || m_LastShootTimestamp == hrClock::time_point())  //单发间隔1s
		{
			SingleShotCtrl(kFeedSemiRPM);
			m_LastShootTimestamp = hrClock::now();
		}
	}
	else if (m_FeedMode == FeedMode::Burst)
	{
		if (interval >= 1000 || m_LastShootTimestamp == hrClock::time_point()) // 三连发间隔1s
		{
			if (shootCnt < kBurstBulletNum)
			{
				SingleShotCtrl(kFeedBurstRPM);
				++shootCnt;
			}
			else
			{
				shootCnt = 0;
				m_LastShootTimestamp = hrClock::now();
			}
				
		}
	}
	else if (m_FeedMode == FeedMode::Auto)
		SingleShotCtrl(kFeedAutoRPM);
}



