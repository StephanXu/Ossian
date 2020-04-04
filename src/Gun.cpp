
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

	//ң�����Ҳ࿪���ϲ�һ�Σ�����Ħ���֣����ϲ�һ�Σ��ر�Ħ����
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

//[TODO] ��ȡ���ؼӳ�RFID״̬����������ٶȼӳ�
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

//[TODO] ����������߼�
void Gun::FeedModeSet()
{
	//���������򲦵���ͣת
	bool overheat = m_CurBulletShotNum.load() >= (m_GunSensorValues.refereeRobotStatus.m_Shooter17HeatLimit
		- m_GunSensorValues.refereePowerHeatData.m_Shooter17Heat) / kHeatPerBullet;
	
	if (m_FricMode == FricMode::Disable)
		m_FeedMode = FeedMode::Stop;
	else if (overheat)
		m_FeedMode = FeedMode::Stop;
	else
	{
		//�ڴ�Ħ���ֵ�����£����ϽǵĲ��֣����� ���������� ����
		int16_t thumbWheelValue = DeadbandLimit(m_GunSensorValues.rc.ch[kShootModeChannel], kGunRCDeadband);
		if (thumbWheelValue < 0)
			m_FeedMode = FeedMode::Semi;
		else if (thumbWheelValue == 0)
			m_FeedMode = FeedMode::Reload;
		else
			m_FeedMode = FeedMode::Auto;
	}
	
	//�������򲦵��ַ�ת
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
 * �����Զ����������ã�����˺���ʱ��΢�����ش��е�
 * ������ת��--->΢�����ؼ�⵽�����뿪--->������ת���͵�--->΢�����ؼ���е���--->������ͣ
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
		if (interval >= 1000)  //�������1s
		{
			SingleShotCtrl(kFeedSemiRPM);
			lastShootTimestamp = hrClock::now();
		}
	}
	else if (m_FeedMode == FeedMode::Burst)
	{
		if (interval >= 1000) // ���������1s
		{
			for (int cnt = 0; cnt < kBurstBulletNum; ++cnt)
				SingleShotCtrl(kFeedBurstRPM);
			lastShootTimestamp = hrClock::now();
		}
	}
	else if (m_FeedMode == FeedMode::Auto)
		SingleShotCtrl(kFeedAutoRPM);
}



