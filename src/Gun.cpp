
#include "Gun.hpp"

int16_t FricCtrlTask::kFricSpeed12 = 0;
int16_t FricCtrlTask::kFricSpeed15 = 0;
int16_t FricCtrlTask::kFricSpeed18 = 0;
int16_t FricCtrlTask::kFricSpeed30 = 0;

int16_t FeedCtrlTask::kFeedNormalSpeed = 0;
int16_t FeedCtrlTask::kFeedSemiSpeed = 0;
int16_t FeedCtrlTask::kFeedBurstSpeed = 0;
int16_t FeedCtrlTask::kFeedAutoSpeed = 0;

std::array<double, 5> FricCtrlTask::PIDFricSpeedParams;

std::array<double, 5> FeedCtrlTask::PIDFeedAngleParams;
std::array<double, 5> FeedCtrlTask::PIDFeedSpeedParams;

void FricCtrlTask::FricModeSet()
{
	static uint8_t lastSw = kRCSwUp;

	//遥控器右侧开关上拨一次，开启摩擦轮；再上拨一次，关闭摩擦轮
	
	//如果云台失能，则摩擦轮也失能
	if (m_FricSensorValues.gimbalInputSrc == GimbalCtrlTask::GimbalInputSrc::Disable
		|| m_FricSensorValues.gimbalInputSrc == GimbalCtrlTask::GimbalInputSrc::Init)
		m_FricMode = FricMode::Disable;
	else if (m_FricSensorValues.rc.sw[kShootModeChannel] == kRCSwUp || m_FricSensorValues.rc.sw[kShootModeChannel] == kRCSwMid)
	{
		m_FricMode = FricMode::Enable;
		/*switch (m_FricMode)
		{
		case FricMode::Disable:
			m_FricMode = FricMode::Enable; break;
		case FricMode::Enable:
			m_FricMode = FricMode::Disable; break;
		default:
			m_FricMode = FricMode::Disable; break;
		}*/
	}

	lastSw = m_FricSensorValues.rc.sw[kShootModeChannel];
	//m_FricMode = FricMode::Disable;
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
		for (size_t i = 0; i < kNumGunFricMotors; ++i)
		{
			double set = (i == 0 ? -m_FricSpeedSet : m_FricSpeedSet);
			double get = m_RPMFdbFilters[i].Calc(m_FricMotorsStatus.m_RPM[i]);
			//double get = m_FricMotorsStatus.m_RPM[i];
			currentSend[i] = m_PIDFricSpeed[i].Calc(set, get);

			/*SPDLOG_INFO("@PIDFric{}=[$setFC{}={},$getFC{}={},$pidoutFC{}={}]",
				i,
				i,
				set,
				i,
				get,
				i,
				currentSend[i]);*/
		}
	}
	
	m_Gun->SendCurrentToMotorsFric(currentSend);
}

//[TODO] 鼠标键盘射击逻辑
void FeedCtrlTask::FeedModeSet()
{
	//若超热量则拨弹轮停转
	bool overheat = m_CurBulletShotNum.load() >= (m_FeedSensorValues.refereeRobotStatus.m_Shooter17HeatLimit
		- m_FeedSensorValues.refereePowerHeatData.m_Shooter17Heat) / kHeatPerBullet;
	
	//若摩擦轮停转，则拨弹轮停转
	if (/*overheat || */m_FricCtrlTask->Stopped()&&0)
		m_FeedMode = FeedMode::Stop;
	else
	{
		if (m_FeedSensorValues.rc.sw[kShootModeChannel] == kRCSwUp)
			m_FeedMode = FeedMode::Auto;
		else
			m_FeedMode = FeedMode::Stop;
		//在打开摩擦轮的情况下：左上角的波轮，向下 单发，向上 连发
		/*int16_t thumbWheelValue = DeadbandLimit(m_FeedSensorValues.rc.ch[kShootModeChannel], kGunRCDeadband);
		if (thumbWheelValue > 0)
			m_FeedMode = FeedMode::Auto;
		else if (thumbWheelValue == 0)
			m_FeedMode = FeedMode::Stop;
		else
			m_FeedMode = FeedMode::Auto;*/
	}
	
	//若卡弹则拨弹轮反转
	/*bool jammed = m_FeedMode != FeedMode::Stop && m_FeedMotorStatus.m_RPM[Feed] < kFeedJamRPM;
	if (jammed)
		m_FeedMode = FeedMode::Reverse; */

	//SPDLOG_INFO("@FeedMode=[$mode={}]", m_FeedMode);
}

//deltaAngle：拨盘的角度变化量
void FeedCtrlTask::FeedRotateCtrl(bool stop, double deltaAngle)
{
	static int lastEcd = -1;
	static double sumPerCtrlDeltaAngleGet = 0;

	double current = 0;
	if (stop)
		current = 0;
	else
	{
		double perCtrlDeltaAngleGet = 0;
		if (lastEcd >= 0)
			sumPerCtrlDeltaAngleGet += RelativeEcdToRad(m_FeedMotorStatus.m_Encoding[FeedCtrlTask::Feed], lastEcd) / kSpeedToMotorRPMCoef;

		if (fabs(deltaAngle - sumPerCtrlDeltaAngleGet) < 0.1)  //已经到达目标位置
		{
			m_FlagInPosition = true;
			sumPerCtrlDeltaAngleGet = 0;
		}
		else
		{
			m_FlagInPosition = false;
			double speedSet = m_PIDFeedAngle.Calc(deltaAngle, sumPerCtrlDeltaAngleGet);  //拨盘速度
			//std::cerr << deltaAngle << '\t' << sumPerCtrlDeltaAngleGet << '\t' << speedSet << std::endl;
			SPDLOG_INFO("@PIDFeedAngle=[$setFdA={},$getFdA={},$pidoutFdA={}]",
				deltaAngle,
				sumPerCtrlDeltaAngleGet,
				speedSet);

			double rpmSet = speedSet * kSpeedToMotorRPMCoef;
			double rpmGet = m_RPMFdbFilter.Calc(m_FeedMotorStatus.m_RPM[FeedCtrlTask::Feed]);
			current = m_PIDFeedSpeed.Calc(rpmSet, rpmGet);

			SPDLOG_INFO("@PIDFeedSpeed=[$setFdS={},$getFdS={},$pidoutFdS={}]",
				rpmSet,
				rpmGet,
				current
			/*static_cast<int>(m_FeedSensorValues.phototubeStatus.m_Status) * 2000*/);
		}
	}
	lastEcd = m_FeedMotorStatus.m_Encoding[FeedCtrlTask::Feed];
	m_Gun->SendCurrentToMotorFeed(current);
}

//void FeedCtrlTask::AutoReloadCtrl()
//{
//	//如果光电管处无弹，则控制拨弹轮补弹
//	if (m_FeedSensorValues.phototubeStatus.m_Status == PhototubeStatus::NO_BULLET)
//		FeedRotateCtrl(false, kFeedNormalSpeed);
//	else //否则，拨弹轮停止
//		FeedRotateCtrl(true);
//}

/**
 * 由于自动补弹的作用，进入此函数时，微动开关处有弹
 * 拨弹轮转动--->微动开关检测到弹丸离开--->拨弹轮转动送弹--->微动开关检测有弹丸--->拨弹轮停
 */
//void FeedCtrlTask::SingleShotCtrl(int speedSet)
//{
//	FeedRotateCtrl(false, speedSet);
//	////如果光电管处有弹，则控制拨弹轮送弹
//	//if (m_FeedSensorValues.phototubeStatus.m_Status == PhototubeStatus::HAS_BULLET)
//	//	FeedRotateCtrl(false, speedSet);
//	//else //否则，控制拨弹轮补弹
//	//	AutoReloadCtrl();
//}

void FeedCtrlTask::FeedCtrl()
{
	static int shootCnt = 0;
	long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - m_LastShootTimestamp).count();
	
	if (m_FeedMode == FeedMode::Stop)
		FeedRotateCtrl(true);
	/*else if (m_FeedMode == FeedMode::Reload)
		AutoReloadCtrl();
	else if (m_FeedMode == FeedMode::Reverse)
		FeedRotateCtrl(false, kFeedNormalSpeed, true);*/
	else if (m_FeedMode == FeedMode::Semi)
	{
		
		//SingleShotCtrl(kFeedSemiSpeed);
		if (interval >= 2000 || m_LastShootTimestamp == std::chrono::high_resolution_clock::time_point())  //单发间隔2s
		{
			FeedRotateCtrl(false, kAnglePerCell);
			if (m_FlagInPosition)
				m_LastShootTimestamp = std::chrono::high_resolution_clock::now();
		}
	}
	else if (m_FeedMode == FeedMode::Burst)
	{
		if (interval >= 2000 || m_LastShootTimestamp == std::chrono::high_resolution_clock::time_point())  //三连发间隔2s
		{
			if (shootCnt < kBurstBulletNum)
			{
				FeedRotateCtrl(false, kAnglePerCell);
				if (m_FlagInPosition)
					++shootCnt;
			}
			else
			{
				shootCnt = 0;
				if (m_FlagInPosition)
					m_LastShootTimestamp = std::chrono::high_resolution_clock::now();
			}
			
		}
		//SingleShotCtrl(kFeedSemiSpeed);
		//if (interval >= 2000 || m_LastShootTimestamp == hrClock::time_point()) // 三连发间隔2s
		//{
		//	if (shootCnt < kBurstBulletNum)
		//	{
		//		SingleShotCtrl(kFeedBurstSpeed);
		//		++shootCnt;
		//	}
		//	else
		//	{
		//		shootCnt = 0;
		//		m_LastShootTimestamp = hrClock::now();
		//	}
		//		
		//}
	}
	else if (m_FeedMode == FeedMode::Auto)
		FeedRotateCtrl(false, kAnglePerCell);
		//SingleShotCtrl(kFeedAutoSpeed);
}



