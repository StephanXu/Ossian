
#include "Gun.hpp"

double Gun::Fric_Speed_12 = 600;
double Gun::Fric_Speed_15 = 800;
double Gun::Fric_Speed_18 = 1000;
double Gun::Fric_Speed_30 = 1200;




void Gun::FricModeSet()
{
	static uint8_t lastSw = RC_SW_UP;

	//ң�����Ҳ࿪���ϲ�һ�Σ�����Ħ���֣����ϲ�һ�Σ��ر�Ħ����
	if (m_GunSensorValues.rc.sw[SHOOT_MODE_CHANNEL] == RC_SW_UP && lastSw != RC_SW_UP)
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

	lastSw = m_GunSensorValues.rc.sw[SHOOT_MODE_CHANNEL];
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
			m_FricSpeedSet = Fric_Speed_12; break;
		case 15:
			m_FricSpeedSet = Fric_Speed_15; break;
		case 18:
			m_FricSpeedSet = Fric_Speed_18; break;
		case 30:
			m_FricSpeedSet = Fric_Speed_30; break;
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
		currentFricBelow = m_PIDFricSpeed[FricBelow].Calc(m_FricSpeedSet, m_Motors[FricBelow]->Status().m_RPM / 60, 
			std::chrono::high_resolution_clock::now());
		currentFricUpper = m_PIDFricSpeed[FricUpper].Calc(-m_FricSpeedSet, m_Motors[FricUpper]->Status().m_RPM / 60, 
			std::chrono::high_resolution_clock::now());
	}
	m_Motors[FricBelow]->SetVoltage(currentFricBelow);
	m_Motors[FricUpper]->SetVoltage(currentFricUpper);
	m_Motors[FricBelow]->Writer()->PackAndSend();
}

//[TODO] ����������߼�
void Gun::FeedModeSet()
{
	static std::chrono::high_resolution_clock::time_point swDownTimestamp = std::chrono::high_resolution_clock::now();

	//���������򲦵���ͣת
	bool overheat = m_CurBulletShotNum >= (m_GunSensorValues.refereeHeatLimit - m_GunSensorValues.refereeCurHeat) / HEAT_PER_BULLET;
	if (overheat)
		m_FeedMode = FeedMode::Stop;
	else
	{
		//ң�����Ҳ࿪�ش������²����������ٲ����У�����
		//ң�����Ҳ࿪�ش������²���,��ͣ�����£�����
		if (m_GunSensorValues.rc.sw[SHOOT_MODE_CHANNEL] == RC_SW_DOWN)
		{
			long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - swDownTimestamp).count();
			if (interval >= 2000)
				m_FeedMode = FeedMode::Auto;
			else
				m_FeedMode = FeedMode::Semi;
		}
		else
		{
			swDownTimestamp = std::chrono::high_resolution_clock::now();
			m_FeedMode = FeedMode::Reload;
		}
			
	}
	
	//�������򲦵��ַ�ת
	bool jammed = m_FeedMode != FeedMode::Stop && m_Motors[Feed]->Status().m_RPM < FEED_JAM_RPM;
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
		current = m_PIDFeedSpeed.Calc(rpmSet, m_Motors[Feed]->Status().m_RPM, std::chrono::high_resolution_clock::now());
		if (reverse)
			current *= -1;
	}
	 
	m_Motors[Feed]->SetVoltage(current);
	m_Motors[Feed]->Writer()->PackAndSend();
}

void Gun::AutoReloadCtrl()
{
	while (!MicroSwitchStatus())
		FeedRotateCtrl(false, FEED_NORMAL_RPM);

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
	if (m_FeedMode == FeedMode::Stop)
		FeedRotateCtrl(true);
	else if (m_FeedMode == FeedMode::Reload)
		AutoReloadCtrl();
	else if (m_FeedMode == FeedMode::Reverse)
		FeedRotateCtrl(false, FEED_NORMAL_RPM, true);
	else if (m_FeedMode == FeedMode::Semi)
		SingleShotCtrl(FEED_SEMI_RPM);
	else if (m_FeedMode == FeedMode::Burst)
	{
		for (int cnt = 0; cnt < BURST_BULLET_NUM; ++cnt)
			SingleShotCtrl(FEED_BURST_RPM);
	}
	else if (m_FeedMode == FeedMode::Auto)
		SingleShotCtrl(FEED_AUTO_RPM);
}



