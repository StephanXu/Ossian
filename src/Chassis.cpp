
#include "Chassis.hpp"


double ChassisCtrlTask::kTopWz = 0;
double ChassisCtrlTask::kVxFilterCoef = 0;
double ChassisCtrlTask::kVyFilterCoef = 0;
double ChassisCtrlTask::kRPMFdbFilterCoef = 0;
std::array<double, 5> ChassisCtrlTask::PIDWheelSpeedParams;
std::array<double, 5> ChassisCtrlTask::PIDChassisAngleParams;


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
	/*SPDLOG_INFO("@WheelSpeedSet=[$wheel0={},$wheel1={},$wheel2={},$wheel3={}]", 
		m_WheelSpeedSet(0), m_WheelSpeedSet(1), m_WheelSpeedSet(2), m_WheelSpeedSet(3));*/
}

void ChassisCtrlTask::ChassisPowerCtrlByCurrent()
{
	//double curPwr, curBuf, maxPwr, maxBuf; //这四个量从裁判系统获取

	double curBuf  = m_ChassisSensorValues.refereePowerHeatData.m_ChassisPowerBuffer;
	double curPwr  = m_ChassisSensorValues.refereePowerHeatData.m_ChassisPower;
	double maxBuf = m_ChassisSensorValues.refereeMaxBuf;
	double maxPwr = m_ChassisSensorValues.refereeMaxPwr;   //[TODO] 根据2020裁判系统串口协议修改
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
	double vxChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[kChassisXChannel], kChassisRCDeadband) * kChassisVxRCSen;		// m/s
	double vyChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[kChassisYChannel], kChassisRCDeadband) * kChassisVyRCSen;       // m/s
	if (m_CurChassisMode == Openloop_Z)
		m_WzSet = -m_ChassisSensorValues.rc.ch[kChassisZChannel] * kChassisWzRCSen;
	else if (m_CurChassisMode == Follow_Chassis_Yaw)
		m_AngleSet = ClampLoop(m_AngleSet - m_ChassisSensorValues.rc.ch[kChassisZChannel] * kChassisAngleWzRCSen, -M_PI, M_PI);
	//[TODO] 键盘操作

	//一阶低通滤波代替斜坡函数作为底盘速度输入
	m_VxSet = m_RCInputFilters[0].Calc(vxChannelSet);//0.167
	m_VySet = m_RCInputFilters[1].Calc(vyChannelSet);//0.333

	m_VxSet = DeadbandLimit(m_VxSet, kChassisRCDeadband * kChassisVxRCSen);
	m_VySet = DeadbandLimit(m_VySet, kChassisRCDeadband * kChassisVyRCSen);
}

void ChassisCtrlTask::ChassisModeSet()
{
	if (0/*m_ChassisSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Disable
		|| m_ChassisSensorValues.gimbalStatus.m_CtrlMode == GimbalCtrlMode::Init*/)
	{
		m_CurChassisMode = Disable;
	}
	else if (m_FlagInitChassis)
	{
		m_CurChassisMode = Init;
	}
	else
	{
		switch (m_ChassisSensorValues.rc.sw[kChassisModeChannel])
		{
		case kRCSwUp:
			m_CurChassisMode = Follow_Chassis_Yaw; break;  //Follow_Gimbal_Yaw
		case kRCSwMid:
			m_CurChassisMode = Openloop_Z;  break;
		case kRCSwDown:
			m_CurChassisMode = Disable; break;   //Top
		default:
			m_CurChassisMode = Disable; break;
		}

	}
	
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
			double set2 = m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef;
			double get2 = m_RPMFdbFilters[i].Calc(m_MotorsStatus.m_RPM[i]);
			m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(set2, get2);
			//SPDLOG_INFO("@MotorSpeed{}=[$rpm{}={}]", i, i, m_Motors[i]->Get().m_RPM);
			/*SPDLOG_INFO("@PIDChassisSpeed{}=[$error={}]", i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef-
				m_Motors[i]->Get().m_RPM);*/
			//SPDLOG_INFO("@RPMAndSet{}=[$rpm{}={},$set{}={}]", i, i, m_Motors[i]->Get().m_RPM, i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef);
			/*SPDLOG_INFO("@PIDChassisSpeed{}=[$set{}={},$get{}={},$pidout{}={}]", 
							i,
							i,
							set2, 
							i,
							get2,
							i,
							m_CurrentSend[i]);*/
			//m_PIDChassisSpeed[i].PrintDetails(i);
		}
	}
	//m_ChassisSensorValues.spCap.m_CapacitorVoltage = 0;
	//如果超级电容快没电了
	if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < kSpCapWarnVoltage)
		ChassisPowerCtrlByCurrent();

	/*for (size_t i = 0; i < kNumChassisMotors; ++i)
		SPDLOG_INFO("@CurrentSend=[$Motor{}={}]", i, m_CurrentSend[i]);*/
	m_Chassis->SendCurrentToMotors(m_CurrentSend);
}

//功率控制：通过减小底盘电机的期望速度来实现
/*
void ChassisCtrlTask::ChassisCtrl()
{
	if (m_CurChassisMode == Disable)
		m_CurrentSend.fill(0);
	else
	{
		CalcWheelSpeedTarget();
		auto CalcCurrent = [this]()->void
		{
			for (size_t i = 0; i < m_Motors.size(); ++i)
			{
				m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(
					m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef,
					m_Motors[i]->Get().m_RPM,
					hrClock::now());
				//SPDLOG_INFO("@PIDChassisSpeed{}=[$set={},$get={},$pidout={}]", i, m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef, m_Motors[i]->Get().m_RPM, m_CurrentSend[i]);
			}
		};
		m_ChassisSensorValues.spCap.m_CapacitorVoltage = 0;
		SPDLOG_INFO("@Capacitor=[$inputV={},$curV={},$inputC={},$targetP={}]", 
			m_ChassisSensorValues.spCap.m_InputVoltage, m_ChassisSensorValues.spCap.m_CapacitorVoltage, 
			m_ChassisSensorValues.spCap.m_TestCurrent, m_ChassisSensorValues.spCap.m_TargetPower);
		//如果超级电容快没电了
		if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < kSpCapWarnVoltage)
		{
			for (int cnt = 0; ; ++cnt)
			{
				m_WheelSpeedSet *= 0.9;
				CalcCurrent();
				double totalCurrent = 0;
				std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [&totalCurrent](double x) {
					totalCurrent += fabs(x); });
				if (totalCurrent * m_ChassisSensorValues.refereePowerHeatData.m_ChassisVolt < 
					m_ChassisSensorValues.refereeMaxPwr)
					break;
				if (cnt >= 10)
				{
					std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [this, totalCurrent](double& x) {
						x = x / totalCurrent * m_ChassisSensorValues.refereeMaxPwr / 
							m_ChassisSensorValues.refereePowerHeatData.m_ChassisVolt; });
					break;
				}
			}
		}
		else
			CalcCurrent();
	}

	for (size_t i = 0; i < m_Motors.size(); ++i)
		SPDLOG_INFO("@CurrentSend=[$Motor{}={}]", i, m_CurrentSend[i]);

	for (size_t i = 0; i < m_Motors.size(); ++i)
		m_Motors[i]->SetVoltage(m_CurrentSend[i]);
	m_Motors[LR]->Writer()->PackAndSend();
}
*/
void ChassisCtrlTask::ChassisExpAxisSpeedSet()
{
	//[TODO] 检验三角函数的符号
	if (m_CurChassisMode == Disable || m_CurChassisMode == Init)
		m_VxSet = m_VySet = m_WzSet = 0;
	else if (m_CurChassisMode == Follow_Gimbal_Yaw)
	{
		RCToChassisSpeed();
		double cosine = cos(-m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double sine = sin(-m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double vx = m_VxSet * cosine + m_VySet * sine;
		double vy = -m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		m_WzSet = -m_PIDChassisAngle.Calc(m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis, 0); //符号为负？
	}
	else if (m_CurChassisMode == Follow_Chassis_Yaw)
	{
		RCToChassisSpeed();		
		double deltaAngle = ClampLoop(m_AngleSet - m_ChassisSensorValues.imu.m_Yaw, -M_PI, M_PI);
		m_WzSet = m_PIDChassisAngle.Calc(deltaAngle, 0); //符号为负？
		SPDLOG_INFO("@PIDChassisYawAngle=[$set={},$get={},$pidout={}]", deltaAngle, 0, m_WzSet);
		m_VxSet = Clamp(m_VxSet, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(m_VySet, -kChassisVyLimit, kChassisVyLimit);
	}
	else if (m_CurChassisMode == Top)
	{
		RCToChassisSpeed();
		double cosine = cos(-m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double sine = sin(-m_ChassisSensorValues.gimbalStatus.m_RelativeAngleToChassis);
		double vx = m_VxSet * cosine + m_VySet * sine;
		double vy = -m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		m_WzSet = kTopWz;
	}
	else if (m_CurChassisMode == Openloop_Z)
	{
		RCToChassisSpeed();
		m_VxSet = Clamp(m_VxSet, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(m_VySet, -kChassisVyLimit, kChassisVyLimit);

		//SPDLOG_INFO("@VSet=[$VxSet={},$VySet={},$WzSet={}]", m_VxSet, m_VySet, m_WzSet);
	}
}
