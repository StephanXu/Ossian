
#include "Gun.hpp"

int16_t Gun::kFricSpeed12 = 0;
int16_t Gun::kFricSpeed15 = 0;
int16_t Gun::kFricSpeed18 = 0;
int16_t Gun::kFricSpeed30 = 0;

int16_t Gun::kFeedNormalRPM = 0;
int16_t Gun::kFeedSemiRPM = 0;
int16_t Gun::kFeedBurstRPM = 0;
int16_t Gun::kFeedAutoRPM = 0;

std::array<double, 5> Gun::PIDFricSpeedParams;
std::array<double, 5> Gun::PIDFeedSpeedParams;

void Gun::FricModeSet()
{
	static uint8_t lastSw = kRCSwUp;

	//遥控器右侧开关上拨一次，开启摩擦轮；再上拨一次，关闭摩擦轮
	if (m_GunSensorValues.rc.sw[kShootModeChannel] == kRCSwUp && lastSw != kRCSwUp)
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
	else if (m_GunSensorValues.gimbalInputSrc == Gimbal::GimbalInputSrc::Disable)
		m_FricMode = FricMode::Disable;
	else
		m_FricMode = FricMode::Disable;

	lastSw = m_GunSensorValues.rc.sw[kShootModeChannel];
}

//[TODO] 读取场地加成RFID状态，叠加射击速度加成
void Gun::FricExpSpeedSet()
{
	if (m_FricMode == FricMode::Disable)
		m_FricSpeedSet = 0;
	else if (m_FricMode == FricMode::Enable)
	{
		switch (m_GunSensorValues.shooter_heat0_speed_limit)
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

void Gun::FricCtrl()
{
	double currentFricBelow = 0, currentFricUpper = 0;

	if (m_FricMode == FricMode::Disable)
		currentFricBelow = currentFricUpper = 0;
	else
	{
		currentFricBelow = m_PIDFricSpeed[FricBelow].Calc(m_FricSpeedSet, m_Motors[FricBelow]->Status().m_RPM, 
			hrClock::now());
		currentFricUpper = m_PIDFricSpeed[FricUpper].Calc(-m_FricSpeedSet, m_Motors[FricUpper]->Status().m_RPM, 
			hrClock::now());
	}
	m_Motors[FricBelow]->SetVoltage(currentFricBelow);
	m_Motors[FricUpper]->SetVoltage(currentFricUpper);
	m_Motors[FricBelow]->Writer()->PackAndSend();
}

//[TODO] 鼠标键盘射击逻辑
void Gun::FeedModeSet()
{
	//若超热量则拨弹轮停转
	bool overheat = m_CurBulletShotNum.load() >= (m_GunSensorValues.refereeRobotStatus.m_Shooter17HeatLimit
		- m_GunSensorValues.refereePowerHeatData.m_Shooter17Heat) / kHeatPerBullet;
	
	if (m_FricMode == FricMode::Disable)
		m_FeedMode = FeedMode::Stop;
	else if (overheat)
		m_FeedMode = FeedMode::Stop;
	else
	{
		//在打开摩擦轮的情况下：左上角的波轮，向下 单发，向上 连发
		int16_t thumbWheelValue = DeadbandLimit(m_GunSensorValues.rc.ch[kShootModeChannel], kGunRCDeadband);
		if (thumbWheelValue < 0)
			m_FeedMode = FeedMode::Semi;
		else if (thumbWheelValue == 0)
			m_FeedMode = FeedMode::Reload;
		else
			m_FeedMode = FeedMode::Auto;
	}
	
	//若卡弹则拨弹轮反转
	bool jammed = m_FeedMode != FeedMode::Stop && m_Motors[Feed]->Status().m_RPM < kFeedJamRPM;
	if (jammed)
		m_FeedMode = FeedMode::Reverse; 
}

void Gun::FeedRotateCtrl(bool stop, int rpmSet, bool reverse)
{
	double current = 0;
	if (stop)
		current = 0;
	else
	{
		current = m_PIDFeedSpeed.Calc(rpmSet, m_Motors[Feed]->Status().m_RPM, hrClock::now());
		if (reverse)
			current *= -1;
	}
	 
	m_Motors[Feed]->SetVoltage(current);
	m_Motors[Feed]->Writer()->PackAndSend();
}

void Gun::AutoReloadCtrl()
{
	while (!MicroSwitchStatus())
		FeedRotateCtrl(false, kFeedNormalRPM);

	FeedRotateCtrl(true);
}

/**
 * 由于自动补弹的作用，进入此函数时，微动开关处有弹
 * 拨弹轮转动--->微动开关检测到弹丸离开--->拨弹轮转动送弹--->微动开关检测有弹丸--->拨弹轮停
 */
void Gun::SingleShotCtrl(int rpmSet)
{
	while (MicroSwitchStatus())
		FeedRotateCtrl(false, rpmSet);
	
	AutoReloadCtrl();
}

void Gun::FeedCtrl()
{
	static hrClock::time_point lastShootTimestamp;
	long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
						 hrClock::now() - lastShootTimestamp).count();
	
	if (m_FeedMode == FeedMode::Stop)
		FeedRotateCtrl(true);
	else if (m_FeedMode == FeedMode::Reload)
		AutoReloadCtrl();
	else if (m_FeedMode == FeedMode::Reverse)
		FeedRotateCtrl(false, kFeedNormalRPM, true);
	else if (m_FeedMode == FeedMode::Semi)
	{
		if (interval >= 1000)  //单发间隔1s
		{
			SingleShotCtrl(kFeedSemiRPM);
			lastShootTimestamp = hrClock::now();
		}
	}
	else if (m_FeedMode == FeedMode::Burst)
	{
		if (interval >= 1000) // 三连发间隔1s
		{
			for (int cnt = 0; cnt < kBurstBulletNum; ++cnt)
				SingleShotCtrl(kFeedBurstRPM);
			lastShootTimestamp = hrClock::now();
		}
	}
	else if (m_FeedMode == FeedMode::Auto)
		SingleShotCtrl(kFeedAutoRPM);
}



