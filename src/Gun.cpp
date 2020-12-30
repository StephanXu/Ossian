#include "Gun.hpp"

int16_t FricCtrlTask::kFricSpeed15 = 0;
int16_t FricCtrlTask::kFricSpeed18 = 0;
int16_t FricCtrlTask::kFricSpeed22 = 0;
int16_t FricCtrlTask::kFricSpeed30 = 0;

//int16_t FeedCtrlTask::kFeedNormalSpeed = 0;
//int16_t FeedCtrlTask::kFeedSemiSpeed = 0;
//int16_t FeedCtrlTask::kFeedBurstSpeed = 0;
//int16_t FeedCtrlTask::kFeedAutoSpeed = 0;

std::array<double, 5> FricCtrlTask::PIDFricSpeedParams;

std::array<double, 5> FeedCtrlTask::PIDFeedAngleParams;
std::array<double, 5> FeedCtrlTask::PIDFeedSpeedParams;

void FricCtrlTask::FricModeSet()
{
	static uint16_t lastKeyboard;

	if (m_FricSensorValues.keyboardMode)
	{
		//键鼠
		if (m_FricSensorValues.rc.keyboard & kKeyboardMapFric.at("FricOn") && !(lastKeyboard & kKeyboardMapFric.at("FricOn")))
		{
			if (m_LastFricMode == FricMode::Disable)
				m_CurFricMode = FricMode::Enable;
			else if (m_LastFricMode == FricMode::Enable)
				m_CurFricMode = FricMode::Disable;
		}
		else
			m_CurFricMode = m_LastFricMode;
	}
	else
	{
		//遥控器模式下，摩擦轮常开
		m_CurFricMode = FricMode::Enable;
	}

	//如果云台失能，则摩擦轮也失能
	if (m_FricSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Disable
		|| m_FricSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Init)
	{
		m_CurFricMode = FricMode::Disable;
	}

	m_LastFricMode = m_CurFricMode;
	lastKeyboard = m_FricSensorValues.rc.keyboard;
}

//[TODO] 读取场地加成RFID状态，叠加射击速度加成
void FricCtrlTask::FricExpSpeedSet()
{
	if (m_CurFricMode == FricMode::Disable)
		m_FricSpeedSet = 0;
	else if (m_CurFricMode == FricMode::Enable)
	{
		switch (m_FricSensorValues.refereeRobotStatus.m_Shooter17SpeedLimit)
		{
		case 15:
			m_FricSpeedSet = kFricSpeed15; break;
		case 18:
			m_FricSpeedSet = kFricSpeed18; break;
		case 22:
			m_FricSpeedSet = kFricSpeed22; break;
		case 30:
			m_FricSpeedSet = kFricSpeed30; break;
		default:
		{
			std::cerr << "[Fric] RefereeRobotStatus.m_Shooter17SpeedLimit Load Failed = " 
				<< m_FricSensorValues.refereeRobotStatus.m_Shooter17SpeedLimit << std::endl;
			m_FricSpeedSet = 0; 
			break;
		}
		}
		m_FricSpeedSet = kFricSpeed15;//!!!!!!!!!!!!
	}
}

void FricCtrlTask::FricCtrl()
{
	static std::array<double, kNumGunFricMotors> currentSend{};
	/*if (m_CurFricMode == FricMode::Disable)
		currentSend.fill(0);
	else*/
	{
		for (size_t i = 0; i < kNumGunFricMotors; ++i)
		{
			double set = (i == 0 ? -m_FricSpeedSet : m_FricSpeedSet);
			//double get = m_RPMFdbFilters[i].Calc(m_FricMotorsStatus.m_RPM[i]);
			double get = m_FricMotorsStatus.m_RPM[i];
			currentSend[i] = m_PIDFricSpeed[i].Calc(set, get);

			/*SPDLOG_INFO("@PIDFric{}=[$setFC{}={},$getFC{}={}]",
				i,
				i,
				set,
				i,
				get);*/
		}
	}
	
	m_Gun->SendCurrentToMotorsFric(currentSend);
}

//[TODO] 鼠标键盘射击逻辑，自瞄发弹逻辑
void FeedCtrlTask::FeedModeSet()
{
	//若超热量则拨弹轮停转
	bool overheat = (m_CurBulletShotNum.load() >= (m_FeedSensorValues.refereeRobotStatus.m_Shooter17HeatLimit
		- m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat) / kHeatPerBullet);
	/*if (overheat)
		std::cerr << "[Feed] OverHeat!!!" << std::endl;*/

	//若摩擦轮停转，则拨弹轮停转
	if (/*overheat 
		|| */m_FeedSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Disable
		|| m_FeedSensorValues.fricStatus.m_Mode == FricMode::Disable
		|| m_FeedSensorValues.fricStatus.m_FlagLowRPM)
	{
		m_FeedMode = FeedMode::Disable;
	}
	else
	{
		if (m_FeedSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Aimbot)
		{
			if (m_FeedSensorValues.gimbalStatus.m_AutoAimShoot)
				m_FeedMode = FeedMode::Auto;
			else 
				m_FeedMode = FeedMode::Disable;
		}
		else if (m_FeedSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Windmill)
		{
			if (m_FeedSensorValues.gimbalStatus.m_AutoAimShoot)
				m_FeedMode = FeedMode::Burst;
			else
				m_FeedMode = FeedMode::Disable;
		}
		else
		{
			if (m_FeedSensorValues.keyboardMode)
			{
				//键鼠模式，单击左键单发，长按左键连发
				static int pressCnt = 0; //统计左键连续按下的时长

				if (m_FeedSensorValues.rc.click[0])
				{
					++pressCnt;
					m_FeedMode = FeedMode::Semi;

					if(pressCnt > 300)
						m_FeedMode = FeedMode::Auto;
				}
				else
				{
					pressCnt = 0;
					m_FeedMode = FeedMode::Disable;
				}
					
			}
			else
			{
				//遥控器模式
				if (m_FeedSensorValues.rc.sw[kShootModeChannel] == kRCSwUp)
					m_FeedMode = FeedMode::Semi;
				else
					m_FeedMode = FeedMode::Disable;

				//在打开摩擦轮的情况下：左上角的波轮，向下 单发，向上 连发
				/*int16_t thumbWheelValue = DeadbandLimit(m_FeedSensorValues.rc.ch[kShootModeChannel], kGunRCDeadband);
				if (thumbWheelValue > 0)
					m_FeedMode = FeedMode::Semi;
				else if (thumbWheelValue == 0)
					m_FeedMode = FeedMode::Stop;
				else
					m_FeedMode = FeedMode::Auto;*/
			}
			
		}
		
	}
	
	//若卡弹则拨弹轮反转
	bool jammed = (!(m_FeedMode == FeedMode::Disable || m_FeedMode == FeedMode::Stop) 
				  && m_FeedMotorStatus.m_RPM[Feed] < kFeedJamRPM);
	if (jammed)
		std::cerr << "[Feed] Jammed!!!" << std::endl;
		//m_FeedMode = FeedMode::Reverse; 
	
	if (m_FeedMode==FeedMode::Semi || m_FeedMode==FeedMode::Burst) 
	{
		if (m_FlagInPosition && m_LastFeedMode == m_FeedMode)
		{
			m_FeedMode = FeedMode::Ready;
			
		}
	}
	//std::cerr << "FeedMode = " << (int)m_FeedMode << std::endl;
	//m_FeedMode = FeedMode::Disable;
	//SPDLOG_INFO("@FeedMode=[$mode={}]", m_FeedMode);
	if (m_FeedMode != FeedMode::Ready)
		m_LastFeedMode = m_FeedMode;
}

//deltaAngle：拨盘的角度变化量
void FeedCtrlTask::FeedRotateCtrl(bool disable, bool continuous, double expDeltaAngle)
{
	double current = 0;
	if (disable)
		current = 0;
	else
	{
		if (m_FlagInPosition)
		{
			m_FeedMotorEcdSumSet = ClampLoop(m_FeedSensorValues.feedAngle + expDeltaAngle, -M_PI, M_PI);
			std::cerr << "Feed In Position" << std::endl;
			m_FlagInPosition = false;
		}
		else
		{
			//std::cerr << m_FeedMotorEcdSumSet << '\t' << m_FeedSensorValues.feedAngle << std::endl;
			double speedSet = m_PIDFeedAngle.Calc(m_FeedMotorEcdSumSet, m_FeedSensorValues.feedAngle);  //拨盘速度，正负号！
				//std::cerr << deltaAngle << '\t' << sumPerCtrlDeltaAngleGet << '\t' << speedSet << std::endl;
			SPDLOG_INFO("@PIDFeedAngle=[$setFdA={},$getFdA={},$pidoutFdA={}]",
				m_FeedMotorEcdSumSet,
				m_FeedSensorValues.feedAngle,
				speedSet);
			
			//if (speedSet > 0)
			//	speedSet = 0;
			if (continuous)
				speedSet = -100;
			else
				speedSet = -20;

			double rpmSet = speedSet * kSpeedToMotorRPMCoef;
			//double rpmGet = m_RPMFdbFilter.Calc(m_FeedMotorStatus.m_RPM[FeedCtrlTask::Feed]);
			current = m_PIDFeedSpeed.Calc(rpmSet, m_FeedMotorStatus.m_RPM[FeedCtrlTask::Feed]);

			SPDLOG_INFO("@PIDFeedSpeed=[$setFdS={},$getFdS={},$pidoutFdS={}]",
				rpmSet,
				m_FeedMotorStatus.m_RPM[FeedCtrlTask::Feed],
				current);
			
			/*if (current > 0)
				current = 0;*/
		}		
		
	}
	
	m_Gun->SendCurrentToMotorFeed(current);
}

void FeedCtrlTask::FeedCtrl()
{
	
	if (m_FeedMode == FeedMode::Disable || m_FeedMode == FeedMode::Ready)
		FeedRotateCtrl(true);
	else if (m_FeedMode == FeedMode::Stop)
		FeedRotateCtrl(false, false, 0);
	/*else if (m_FeedMode == FeedMode::Reload)
		AutoReloadCtrl();
	else if (m_FeedMode == FeedMode::Reverse)
		FeedRotateCtrl(false, kFeedNormalSpeed, true);*/
	else if (m_FeedMode == FeedMode::Semi)
	{
		m_PIDFeedAngle.SetOutputLimit(-15, 15);
		FeedRotateCtrl(false, false, kDeltaAnglePerBullet);
	} 
	else if (m_FeedMode == FeedMode::Auto)
	{
		m_PIDFeedAngle.SetOutputLimit(-60, 60);
		FeedRotateCtrl(false, true, kDeltaAnglePerBullet);
	}
	else if (m_FeedMode == FeedMode::Burst)
	{
		m_PIDFeedAngle.SetOutputLimit(-30, 30);
		FeedRotateCtrl(false, false, kDeltaAnglePerBullet * kBurstBulletNum);
	}
}



