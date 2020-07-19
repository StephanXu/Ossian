
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

	//ң�����Ҳ࿪���ϲ�һ�Σ�����Ħ���֣����ϲ�һ�Σ��ر�Ħ����
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
	//�����̨ʧ�ܣ���Ħ����Ҳʧ��
	/*else if (m_FricSensorValues.gimbalInputSrc == GimbalCtrlTask::GimbalInputSrc::Disable)
		m_FricMode = FricMode::Disable;*/

	lastSw = m_FricSensorValues.rc.sw[kShootModeChannel];
}

//[TODO] ��ȡ���ؼӳ�RFID״̬����������ٶȼӳ�
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
		currentSend[FricBelow] = m_PIDFricSpeed[FricBelow].Calc(-m_FricSpeedSet, m_FricMotorsStatus.m_RPM[FricBelow]);
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

//[TODO] ����������߼�
void FeedCtrlTask::FeedModeSet()
{
	//���������򲦵���ͣת
	bool overheat = m_CurBulletShotNum.load() >= (m_FeedSensorValues.refereeRobotStatus.m_Shooter17HeatLimit
		- m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat) / kHeatPerBullet;
	
	//��Ħ����ͣת���򲦵���ͣת
	if (overheat || m_FricCtrlTask->Stopped())
		m_FeedMode = FeedMode::Stop;
	else
	{
		//�ڴ�Ħ���ֵ�����£����ϽǵĲ��֣����� ���������� ����
		int16_t thumbWheelValue = DeadbandLimit(m_FeedSensorValues.rc.ch[kShootModeChannel], kGunRCDeadband);
		if (thumbWheelValue < 0)
			m_FeedMode = FeedMode::Semi;
		else if (thumbWheelValue == 0)
			m_FeedMode = FeedMode::Reload;
		else
			m_FeedMode = FeedMode::Auto;
	}
	
	//�������򲦵��ַ�ת
	bool jammed = m_FeedMode != FeedMode::Stop && m_FeedMotorStatus.m_RPM[Feed] < kFeedJamRPM;
	if (jammed)
		m_FeedMode = FeedMode::Reverse; 
}

void FeedCtrlTask::FeedRotateCtrl(bool stop, int rpmSet, bool reverse)
{
	double current = 0;
	if (stop)
		current = 0;
	else
	{
		current = m_PIDFeedSpeed.Calc(rpmSet, m_FeedMotorStatus.m_RPM[Feed]);
		if (reverse)
			current *= -1;
	}
	
	m_Gun->SendCurrentToMotorFeed(current);
}

void FeedCtrlTask::AutoReloadCtrl()
{
	while (!MicroSwitchStatus())
		FeedRotateCtrl(false, kFeedNormalRPM);

	FeedRotateCtrl(true);
}

/**
 * �����Զ����������ã�����˺���ʱ��΢�����ش��е�
 * ������ת��--->΢�����ؼ�⵽�����뿪--->������ת���͵�--->΢�����ؼ���е���--->������ͣ
 */
void FeedCtrlTask::SingleShotCtrl(int rpmSet)
{
	while (MicroSwitchStatus())
		FeedRotateCtrl(false, rpmSet);
	
	AutoReloadCtrl();
}

void FeedCtrlTask::FeedCtrl()
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



