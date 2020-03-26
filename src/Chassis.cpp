
#include "Chassis.hpp"


double Chassis::kTopWz = 0;
std::array<double, 5> Chassis::PIDWheelSpeedParams;
std::array<double, 5> Chassis::PIDChassisAngleParams;

Eigen::Matrix<double, 4, 3> Chassis::m_WheelKinematicMat;


void Chassis::CalcWheelSpeedTarget()
{
	Eigen::Vector3d vSet(m_VxSet, m_VySet, m_WzSet);
	m_WheelSpeedSet = m_WheelKinematicMat * vSet / kWheelRadius; //[4, 3] * [3, 1] --> [4, 1]

	//限制麦轮最大速度
	double maxWheelSpeedItem = m_WheelSpeedSet.maxCoeff();
	if (maxWheelSpeedItem > kWheelSpeedLimit)
	{
		double scaleWheelSpeed = kWheelSpeedLimit / maxWheelSpeedItem;
		m_WheelSpeedSet *= scaleWheelSpeed;
	}
}

void Chassis::ChassisPowerCtrlByCurrent()
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

void Chassis::RCToChassisSpeed()
{
	double vxChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[kChassisXChannel], kChassisRCDeadband) * kChassisVxRCSen;		// m/s
	double vyChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[kChassisYChannel], kChassisRCDeadband) * (-kChassisVyRCSen);    // m/s
	if (m_CurChassisMode == Openloop_Z)
		m_WzSet = m_ChassisSensorValues.rc.ch[kChassisZChannel] * (-kChassisWzRCSen);
	//[TODO] 键盘操作

	//一阶低通滤波代替斜坡函数作为底盘速度输入
	m_VxSet = m_FOFilterVX.Calc(vxChannelSet);
	m_VySet = m_FOFilterVY.Calc(vyChannelSet);
}

void Chassis::ChassisModeSet()
{
	switch (m_ChassisSensorValues.rc.sw[kChassisModeChannel])
	{
	case kRCSwUp:
		m_CurChassisMode = Openloop_Z; break;  //Follow_Gimbal_Yaw
	case kRCSwMid:
		m_CurChassisMode = Disable; break; 
	case kRCSwDown:
		m_CurChassisMode = Top; break; 
	default:
		m_CurChassisMode = Disable; break;
	}
	
}

/*void Chassis::ChassisCtrl()
{
	CalcWheelSpeedTarget();
	for (size_t i = 0; i < 4; ++i)
		m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(m_WheelSpeedSet(i), m_Motors[i]->Status().m_RPM * MOTOR_RPM_TO_WHEEL_SPEED_COEF, m_Motors[i]->TimeStamp());
	
	//如果超级电容快没电了
	if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < kSpCapWarnVoltage)
		ChassisPowerCtrlByCurrent();

	//SendCurrentToMotor();
}*/

//功率控制：通过减小底盘电机的期望速度来实现
void Chassis::ChassisCtrl()
{
	CalcWheelSpeedTarget();
	
	static auto CalcCurrent = [this]()->void
	{
		for (size_t i = 0; i < 4; ++i)
			m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(
				m_WheelSpeedSet(i) * kWheelSpeedToMotorRPMCoef,
				m_Motors[i]->Status().m_RPM / 60,
				std::chrono::high_resolution_clock::now());
	};
	
	//如果超级电容快没电了
	if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < kSpCapWarnVoltage)
	{
		for (int cnt = 0; ; ++cnt)
		{
			m_WheelSpeedSet *= 0.9;
			CalcCurrent();
			double totalCurrent = 0;
			std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [&totalCurrent](double x) {totalCurrent += fabs(x); });
			if (totalCurrent * m_ChassisSensorValues.spCap.m_InputVoltage < m_ChassisSensorValues.refereeMaxPwr)
				break;
			if (cnt >= 10)
			{
				std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [this, totalCurrent](double& x) {x = x / totalCurrent * m_ChassisSensorValues.refereeMaxPwr / m_ChassisSensorValues.spCap.m_InputVoltage; });
				break;
			}
		}
	}
	else
		CalcCurrent();

	std::for_each(m_CurrentSend.begin(), m_CurrentSend.end(), [this](double x, size_t ix = 0) {m_Motors[ix++]->SetVoltage(x); });
	m_Motors[LR]->Writer()->PackAndSend();
}

void Chassis::ChassisExpAxisSpeedSet()
{
	//[TODO] 检验三角函数的符号
	if (m_CurChassisMode == Disable)
		m_VxSet = m_VySet = m_WzSet = 0;
	else if (m_CurChassisMode == Follow_Gimbal_Yaw)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.relativeAngle), sine = sin(m_ChassisSensorValues.relativeAngle);
		double vx = m_VxSet * cosine - m_VySet * sine;
		double vy = m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		m_WzSet = m_PIDChassisAngle.Calc(m_ChassisSensorValues.relativeAngle, 0, std::chrono::high_resolution_clock::now(), true); //符号为负？
	}
	/*else if (m_CurChassisMode == Follow_Chassis_Yaw)
	{
		RCToChassisSpeed();
		m_AngleSet = ClampLoop(m_AngleSet - m_ChassisSensorValues.rc.ch[kChassisZChannel] * kChassisWzRCSen, -M_PI, M_PI);
		double deltaAngle = ClampLoop(m_AngleSet - m_ChassisSensorValues.gyroZ, -M_PI, M_PI);
		m_WzSet = m_PIDChassisAngle.Calc(deltaAngle, 0, std::chrono::high_resolution_clock::now(), true); //符号为负？
		m_VxSet = Clamp(m_VxSet, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(m_VySet, -kChassisVyLimit, kChassisVyLimit);
	}*/
	else if (m_CurChassisMode == Top)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.relativeAngle), sine = sin(m_ChassisSensorValues.relativeAngle);
		double vx = m_VxSet * cosine - m_VySet * sine;
		double vy = m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(vy, -kChassisVyLimit, kChassisVyLimit);
		m_WzSet = kTopWz;
	}
	else if (m_CurChassisMode == Openloop_Z)
	{
		RCToChassisSpeed();
		m_VxSet = Clamp(m_VxSet, -kChassisVxLimit, kChassisVxLimit);
		m_VySet = Clamp(m_VxSet, -kChassisVyLimit, kChassisVyLimit);
	}
}
