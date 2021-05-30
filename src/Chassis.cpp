
#include "Chassis.hpp"


double ChassisCtrlTask::kTopWz = 0;
double ChassisCtrlTask::kVxFilterCoef = 0;
double ChassisCtrlTask::kVyFilterCoef = 0;
double ChassisCtrlTask::kRPMFdbFilterCoef = 0;
std::array<double, 5> ChassisCtrlTask::PIDWheelSpeedParams;
std::array<double, 5> ChassisCtrlTask::PIDChassisAngleParams;


void ChassisCtrlTask::FillClientGraphics()
{
	auto& textStyleRef = m_ClientGraphicTextSpCapStatus->GetStyleRef();
	textStyleRef.m_Color = 5;
	textStyleRef.m_FontSize = 40;
	textStyleRef.m_Width = 3;
	textStyleRef.m_StartX = 787;
	textStyleRef.m_StartY = 949; 
	textStyleRef.m_Text = fmt::format("SPCAP: {:.1f}%", (m_ChassisSensorValues.spCap.m_CapacitorVoltage - kSpCapWarnVoltage)
															/ (24.0 - kSpCapWarnVoltage));
	m_ClientGraphicTextSpCapStatus->Save();

	//当前版本客户端尚未实现以下代码的功能
	/*auto& floatStyleRef = m_ClientGraphicValueSpCapStatus->GetStyleRef();
	floatStyleRef.m_Color = 5;
	floatStyleRef.m_FontSize = 40;
	floatStyleRef.m_Width = 3;
	floatStyleRef.m_StartX = 420;
	floatStyleRef.m_StartY = 700;
	floatStyleRef.m_Value = m_ChassisSensorValues.spCap.m_CapacitorVoltage - kSpCapWarnVoltage;
	floatStyleRef.m_Precision = 1;
	m_ClientGraphicValueSpCapStatus->Save();*/
}

void ChassisCtrlTask::CalcWheelSpeedTarget()
{
	Eigen::Vector3d vSet(m_VxSet, m_VySet, m_WzSet);  //底盘三轴运动速度期望 m/s
	m_WheelSpeedSet = m_WheelKinematicMat * vSet / kWheelRadius / (2.0 * M_PI) * 60; //[4, 3] * [3, 1] --> [4, 1]  轮子转速期望rpm
	//m_WheelSpeedSet(2) *= -1, m_WheelSpeedSet(3) *= -1;  //3,4号电机转向与轮子相反
	//限制麦轮最大速度
	/*double maxWheelSpeedItem = m_WheelSpeedSet.maxCoeff();
	if (maxWheelSpeedItem > kWheelSpeedLimit)
	{
		double scaleWheelSpeed = kWheelSpeedLimit / maxWheelSpeedItem;
		m_WheelSpeedSet *= scaleWheelSpeed;
	}*/
	/*SPDLOG_TRACE("@WheelSpeedSet=[$wheel0={},$wheel1={},$wheel2={},$wheel3={}]", 
		m_WheelSpeedSet(0), m_WheelSpeedSet(1), m_WheelSpeedSet(2), m_WheelSpeedSet(3));*/
}

void ChassisCtrlTask::ChassisPowerCtrlByCurrent()
{
	//double curPwr, curBuf, maxPwr, maxBuf; //这四个量从裁判系统获取
	/*double curPwr = 0;
	for (size_t i = 0; i < kNumChassisMotors; ++i)
		curPwr += 0.00000394047917046875 * fabs(m_MotorsStatus.m_Current[i]) * fabs(m_MotorsStatus.m_RPM[i]) + 1.4;
	SPDLOG_TRACE("@ChassisPower=[$pwr={}]", curPwr);*/

	double curBuf  = m_ChassisSensorValues.refereePowerHeatData.m_ChassisPowerBuffer;
	double curPwr  = m_ChassisSensorValues.refereePowerHeatData.m_ChassisPower;
	double maxBuf = 60; // [TODO] 根据飞坡增益修改 250J
	double maxPwr = m_ChassisSensorValues.refereeRobotStatus.m_ChassisPowerLimit;
	double warnBuf = maxBuf * 0.9;
	double warnPwr = maxPwr * 0.9;
	
	double scalePwr;  //功率缩小系数
	double limitCurrent; //底盘电机总电流上限
	//缓冲能量小于预警值则说明功率超限
	if (curBuf < maxBuf)
	{
		if (curBuf > warnBuf * 0.1)
			scalePwr = curBuf / warnBuf;
		else
			scalePwr = 0.1;
		limitCurrent = kBufferTotalCurrentLimit * scalePwr;
	}
	else
	{
		if (curPwr > warnPwr)
		{
			if (curPwr < maxPwr)
				scalePwr = (maxPwr - curPwr) / (maxPwr - warnPwr);
			else
				scalePwr = 0;
		}
		else
			scalePwr = 1;
		limitCurrent = kBufferTotalCurrentLimit + kPowerTotalCurrentLimit * scalePwr;
	}
	limitCurrent = fabs(limitCurrent);

	double totalCurrent = 0;
	std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [&totalCurrent](double x) {totalCurrent += fabs(x); });
	if (totalCurrent > limitCurrent)
	{
		double scaleCurrent = limitCurrent / totalCurrent;
		std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [scaleCurrent](double& x) {x *= scaleCurrent; });
	}
}

void ChassisCtrlTask::RCToChassisSpeed()
{
	bool keyboardMode = (m_ChassisSensorValues.rc.sw[0] == kRCSwUp && m_ChassisSensorValues.rc.sw[1] == kRCSwUp);
	double vxChannelSet = 0, vyChannelSet = 0;
	if (keyboardMode)
	{
		//键鼠
		if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("Forward"))
		{
			if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("SpeedUp"))
				vxChannelSet = -kChassisVxBoost;
			else
				vxChannelSet = -kChassisVxNormal;
		}
		else if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("Backward"))
		{
			if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("SpeedUp"))
				vxChannelSet = kChassisVxBoost;
			else
				vxChannelSet = kChassisVxNormal;
		}
		else
			vxChannelSet = 0;

		if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("Leftward"))
		{
			if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("SpeedUp"))
				vyChannelSet = kChassisVyBoost;
			else
				vyChannelSet = kChassisVyNormal;
		}
		else if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("Rightward"))
		{
			if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("SpeedUp"))
				vyChannelSet = -kChassisVyBoost;
			else
				vyChannelSet = -kChassisVyNormal;
		}
		else
			vyChannelSet = 0;
	}
	else
	{
		//遥控器
		vxChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[kChassisXChannel], kChassisRCDeadband) * kChassisVxRCSen;	// m/s
		vyChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[kChassisYChannel], kChassisRCDeadband) * kChassisVyRCSen;   // m/s
		if (m_CurChassisMode == Openloop_Z)
			m_WzSet = -m_ChassisSensorValues.rc.ch[kChassisZChannel] * kChassisWzRCSen;
		else if (m_CurChassisMode == Follow_Chassis_Yaw)
			m_AngleSet = ClampLoop(m_AngleSet - m_ChassisSensorValues.rc.ch[kChassisZChannel] * kChassisAngleWzRCSen, -M_PI, M_PI);
	}
	
	

	//一阶低通滤波代替斜坡函数作为底盘速度输入
	if (vxChannelSet == 0)
	{
		m_VxSet = 0;
		m_RCInputFilters[0].Reset();
	}
	else
	{
		m_VxSet = m_RCInputFilters[0].Calc(vxChannelSet);//0.167
		m_VxSet = DeadbandLimit(m_VxSet, kChassisRCDeadband * kChassisVxRCSen);
	}

	if (vyChannelSet == 0)
	{
		m_VySet = 0;
		m_RCInputFilters[1].Reset();
	}
	else
	{
		m_VySet = m_RCInputFilters[1].Calc(vyChannelSet);//0.333
		m_VySet = DeadbandLimit(m_VySet, kChassisRCDeadband * kChassisVyRCSen);
	}

}

void ChassisCtrlTask::ChassisModeSet()
{
	static uint16_t lastKeyboard;

	if (m_ChassisSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Disable
		|| m_ChassisSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Init)
	{
		m_CurChassisMode = Disable;
	}
	else if (m_FlagInitChassis)
	{
		m_CurChassisMode = Init;
	}
	else
	{
		if (m_ChassisSensorValues.keyboardMode)
		{
			//键鼠
			if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("TopMode"))
			{
				if (!(lastKeyboard & kKeyboardMapChassis.at("TopMode")))
				{
					if (m_LastChassisMode != Top)
						m_CurChassisMode = Top;
					else
						m_CurChassisMode = Follow_Gimbal_Yaw;
				}
			}
			else if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("ShieldMode"))
			{
				m_CurChassisMode = Shield;
			}
			else if (m_ChassisSensorValues.rc.keyboard & kKeyboardMapChassis.at("OpenLoopZMode"))
			{
				if (!(lastKeyboard & kKeyboardMapChassis.at("OpenLoopZMode")))
				{
					if (m_LastChassisMode != Openloop_Z)
						m_CurChassisMode = Openloop_Z;
					else
						m_CurChassisMode = Follow_Gimbal_Yaw;
				}
			}
			else if (m_LastChassisMode == Top)
			{
				m_CurChassisMode = Top;
			}
			else
			{
				m_CurChassisMode = Follow_Gimbal_Yaw;
			}
		}
		else
		{
			//遥控器
			switch (m_ChassisSensorValues.rc.sw[kChassisModeChannel])
			{
			case kRCSwUp:
				m_CurChassisMode = Follow_Gimbal_Yaw; break;  //Shield Follow_Gimbal_Yaw
			case kRCSwMid:
				m_CurChassisMode = Follow_Gimbal_Yaw;  break;
			case kRCSwDown:
				m_CurChassisMode = Disable; break;   //Top
			default:
				m_CurChassisMode = Disable; break;
			}
		}
	}

	if (m_LastChassisMode == Top && m_CurChassisMode != Top)
	{
		m_TopWzFilter.Reset();
		for (size_t i = 0; i < kNumChassisMotors; ++i)
		{
			m_PIDChassisSpeed[i].Reset();
		}
	}
		

	m_LastChassisMode = m_CurChassisMode;
	lastKeyboard = m_ChassisSensorValues.rc.keyboard;
}

void ChassisCtrlTask::ChassisCtrl()
{
	if (m_CurChassisMode == Disable || m_CurChassisMode == Init)
		m_CurrentSend.fill(0);
	else
	{
		CalcWheelSpeedTarget();
		for (size_t i = 0; i < kNumChassisMotors; ++i)
		{
			double set = m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef; 
			//double get = m_RPMFdbFilters[i].Calc(m_MotorsStatus.m_RPM[i]);
			double get = m_MotorsStatus.m_RPM[i];
			m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(set, get);
			//SPDLOG_TRACE("@MotorSpeed{}=[$rpm{}={}]", i, i, m_Motors[i]->Get().m_RPM);
			/*SPDLOG_TRACE("@PIDChassisSpeed{}=[$error={}]", i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef-
				m_Motors[i]->Get().m_RPM);*/
			//SPDLOG_TRACE("@RPMAndSet{}=[$rpm{}={},$set{}={}]", i, i, m_Motors[i]->Get().m_RPM, i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef);
			if (i == 0)
			{
				SPDLOG_TRACE("@PIDChassisSpeed{}=[$set0{}={},$get0{}={},$pidout0{}={}]",
					i,
					i,
					set,
					i,
					get,
					i,
					m_CurrentSend[i]);
			}	
			//m_PIDChassisSpeed[i].PrintDetails(i);
		}
	}
	//m_ChassisSensorValues.spCap.m_CapacitorVoltage = 0;
	////如果超级电容快没电了
	//if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < kSpCapWarnVoltage)
	//	ChassisPowerCtrlByCurrent();

	//for单项赛：如果快死了
	if(m_ChassisSensorValues.refereeRobotStatus.m_RemainHp < m_ChassisSensorValues.refereeRobotStatus.m_MaxHp * 0.5
		|| m_ChassisSensorValues.rc.sw[kChassisModeChannel] == kRCSwUp)
		ChassisPowerCtrlByCurrent();

	/*for (size_t i = 0; i < kNumChassisMotors; ++i)
		SPDLOG_TRACE("@CurrentSend=[$Motor{}={}]", i, m_CurrentSend[i]);*/
	SPDLOG_TRACE("@RefereePowerHeatData=[$Power={},$Buffer={},$PowerLimit={}]",
		m_ChassisSensorValues.refereePowerHeatData.m_ChassisPower,
		m_ChassisSensorValues.refereePowerHeatData.m_ChassisPowerBuffer,
		m_ChassisSensorValues.refereeRobotStatus.m_ChassisPowerLimit);

	SPDLOG_TRACE("@SpCap=[$CapPower={}]",
		m_ChassisSensorValues.spCap.m_CapacitorVoltage);

	m_Chassis->SendCurrentToMotors(m_CurrentSend);
}

//功率控制：通过减小底盘电机的期望速度来实现
//void ChassisCtrlTask::ChassisCtrl()
//{
//	if (m_CurChassisMode == Disable || m_CurChassisMode == Init)
//		m_CurrentSend.fill(0);
//	else
//	{
//		static auto CalcCurrent = [this]()->void
//		{
//			for (size_t i = 0; i < kNumChassisMotors; ++i)
//			{
//				double set = m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef;
//				double get = m_MotorsStatus.m_RPM[i];
//				//double get = m_RPMFdbFilters[i].Calc(m_MotorsStatus.m_RPM[i]);
//				m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(set, get);
//				//SPDLOG_TRACE("@MotorSpeed{}=[$rpm{}={}]", i, i, m_Motors[i]->Get().m_RPM);
//				/*SPDLOG_TRACE("@PIDChassisSpeed{}=[$error={}]", i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef-
//					m_Motors[i]->Get().m_RPM);*/
//					//SPDLOG_TRACE("@RPMAndSet{}=[$rpm{}={},$set{}={}]", i, i, m_Motors[i]->Get().m_RPM, i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef);
//				SPDLOG_TRACE("@PIDChassisSpeed{}=[$set{}={},$get{}={},$pidout{}={}]",
//					i,
//					i,
//					set,
//					i,
//					get,
//					i,
//					m_CurrentSend[i]);
//			}
//		};
//		static constexpr double maxBuf = 60;
//		static constexpr double warnBuf = maxBuf * 0.1;
//		double maxPwr = m_ChassisSensorValues.refereeRobotStatus.m_ChassisPowerLimit;
//		if (m_ChassisSensorValues.refereePowerHeatData.m_ChassisPowerBuffer < warnBuf)
//			maxPwr = (warnBuf - m_ChassisSensorValues.refereePowerHeatData.m_ChassisPowerBuffer) / 0.02; 
//
//		CalcWheelSpeedTarget();
//		//m_ChassisSensorValues.spCap.m_CapacitorVoltage = 0;
//		//如果超级电容快没电了
//		if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < kSpCapWarnVoltage)
//		{
//			for (int cnt = 0; ; ++cnt)
//			{
//				CalcCurrent();
//				double totalMotorPower = 0;
//				for (size_t i = 0; i < kNumChassisMotors; ++i)
//					totalMotorPower += (0.00000394047917046875 * fabs(m_CurrentSend[i]) / 819.2 * fabs(m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef) + 1.4);
//				//SPDLOG_TRACE("@ChassisPower=[$pwr={}]", totalMotorPower);
//				//std::cerr << "[ChassisPower] " << "Max= " << (int)m_ChassisSensorValues.refereeRobotStatus.m_ChassisMaxPower << " SetTot= " << totalMotorPower << std::endl;
//
//				if (totalMotorPower < maxPwr)
//					break;
//				if (cnt >= 10)
//				{
//					std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [this, totalMotorPower, maxPwr](double& x) {
//						x *= (maxPwr / totalMotorPower); });
//					break;
//				}
//
//				m_WheelSpeedSet *= 0.9;
//			}
//		}
//		else
//			CalcCurrent();
//	}
//
//	/*for (size_t i = 0; i < kNumChassisMotors; ++i)
//		std::cerr << m_CurrentSend[i] << '\t';
//	std::cerr<< std::endl;*/
//
//	m_Chassis->SendCurrentToMotors(m_CurrentSend);
//}

void ChassisCtrlTask::ChassisExpAxisSpeedSet()
{
	//[TODO] 检验三角函数的符号
	if (m_CurChassisMode == Disable || m_CurChassisMode == Init)
	{
		m_VxSet = m_VySet = m_WzSet = 0;
	}
	else if (m_CurChassisMode == Follow_Gimbal_Yaw)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double sine = sin(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double vx = m_VxSet * cosine + m_VySet * sine;
		double vy = -m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		m_WzSet = -m_PIDChassisAngle.Calc(0, m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis); //符号为负？
	}
	else if (m_CurChassisMode == Follow_Chassis_Yaw)
	{
		RCToChassisSpeed();		
		double deltaAngle = ClampLoop(m_AngleSet - m_ChassisSensorValues.imu.m_Yaw, -M_PI, M_PI);
		m_WzSet = m_PIDChassisAngle.Calc(deltaAngle, 0); //符号为负？
		//SPDLOG_TRACE("@PIDChassisYawAngle=[$set={},$get={},$pidout={}]", deltaAngle, 0, m_WzSet);
		m_VxSet = Clamp(m_VxSet, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(m_VySet, -kChassisVyLimit, kChassisVyLimit);
	}
	else if (m_CurChassisMode == Top)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double sine = sin(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double vx = m_VxSet * cosine + m_VySet * sine;
		double vy = -m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		//m_WzSet = kTopWz;
		m_WzSet = m_TopWzFilter.Calc(kTopWz);
	}
	else if (m_CurChassisMode == Shield)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double sine = sin(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double vx = m_VxSet * cosine + m_VySet * sine;
		double vy = -m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		m_WzSet = -m_PIDChassisAngle.Calc(M_PI / 4.0, m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis); //符号为负？
	}
	else if (m_CurChassisMode == Openloop_Z)
	{
		RCToChassisSpeed();
		m_VxSet = Clamp(m_VxSet, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(m_VySet, -kChassisVyLimit, kChassisVyLimit);

		//SPDLOG_TRACE("@VSet=[$VxSet={},$VySet={},$WzSet={}]", m_VxSet, m_VySet, m_WzSet);
	}
	
}
