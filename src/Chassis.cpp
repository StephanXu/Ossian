
#include "Chassis.hpp"


double Chassis::Top_Wz = 0;
Eigen::Matrix<double, 4, 3> Chassis::m_WheelKinematicMat;


void Chassis::CalcWheelSpeedTarget()
{
	Eigen::Vector3d vSet(m_VxSet, m_VySet, m_WzSet);
	m_WheelSpeedSet = m_WheelKinematicMat * vSet / WHEEL_RADIUS; //[4, 3] * [3, 1] --> [4, 1]

	//限制麦轮最大速度
	double maxWheelSpeedItem = m_WheelSpeedSet.maxCoeff();
	if (maxWheelSpeedItem > LIMIT_WHEEL_SPEED)
	{
		double scaleWheelSpeed = LIMIT_WHEEL_SPEED / maxWheelSpeedItem;
		m_WheelSpeedSet *= scaleWheelSpeed;
	}
}

void Chassis::ChassisPowerCtrlByCurrent()
{
	//double curPwr, curBuf, maxPwr, maxBuf; //这四个量从裁判系统获取
	double warnBuf = m_ChassisSensorValues.refereeMaxBuf * 0.9;
	double warnPwr = m_ChassisSensorValues.refereeMaxPwr * 0.9;
	
	double scalePwr;  //功率缩小系数
	double limitCurrent; //底盘电机总电流上限
	//缓冲能量小于预警值则说明功率超限
	if (m_ChassisSensorValues.refereeCurBuf < m_ChassisSensorValues.refereeMaxBuf)
	{
		if (m_ChassisSensorValues.refereeCurBuf > warnBuf * 0.1)
			scalePwr = m_ChassisSensorValues.refereeCurBuf / warnBuf;
		else
			scalePwr = 0.1;
		limitCurrent = LIMIT_BUFFER_TOTAL_CURRENT * scalePwr;
	}
	else
	{
		if (m_ChassisSensorValues.refereeCurPwr > warnPwr)
		{
			if (m_ChassisSensorValues.refereeCurPwr < m_ChassisSensorValues.refereeMaxPwr)
				scalePwr = (m_ChassisSensorValues.refereeMaxPwr - m_ChassisSensorValues.refereeCurPwr) / (m_ChassisSensorValues.refereeMaxPwr - warnPwr);
			else
				scalePwr = 0;
		}
		else
			scalePwr = 1;
		limitCurrent = LIMIT_BUFFER_TOTAL_CURRENT + LIMIT_POWER_TOTAL_CURRENT * scalePwr;
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
	double vxChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[CHASSIS_X_CHANNEL], CHASSIS_RC_DEADBAND) * CHASSIS_VX_RC_SEN;		// m/s
	double vyChannelSet = DeadbandLimit(m_ChassisSensorValues.rc.ch[CHASSIS_Y_CHANNEL], CHASSIS_RC_DEADBAND) * (-CHASSIS_VY_RC_SEN);  // m/s
	if (m_CurChassisMode == OPENLOOP_Z)
		m_WzSet = m_ChassisSensorValues.rc.ch[CHASSIS_Z_CHANNEL] * (-CHASSIS_WZ_RC_SEN);
	//[TODO] 键盘操作

	//一阶低通滤波代替斜坡函数作为底盘速度输入
	m_VxSet = m_FOFilterVX.Calc(vxChannelSet);
	m_VySet = m_FOFilterVY.Calc(vyChannelSet);
}

void Chassis::ChassisModeSet()
{
	switch (m_ChassisSensorValues.rc.sw[CHASSIS_MODE_CHANNEL])
	{
	case RC_SW_UP:
		m_CurChassisMode = FOLLOW_GIMBAL_YAW; break;
	case RC_SW_MID:
		m_CurChassisMode = TOP; break;
	case RC_SW_DOWN:
		m_CurChassisMode = OPENLOOP_Z; break;
	default:
		m_CurChassisMode = DISABLE; break;
	}
	
}

/*void Chassis::ChassisCtrl()
{
	CalcWheelSpeedTarget();
	for (size_t i = 0; i < 4; ++i)
		m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(m_WheelSpeedSet(i), m_Motors[i]->Status().m_RPM * MOTOR_RPM_TO_WHEEL_SPEED_COEF, m_Motors[i]->TimeStamp());
	
	//如果超级电容快没电了
	if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < SPCAP_WARN_VOLTAGE)
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
				m_WheelSpeedSet(i) * WHEEL_SPEED_TO_MOTOR_RPM_COEF,
				m_Motors[i]->Status().m_RPM,
				m_Motors[i]->TimeStamp());
	};
	
	//如果超级电容快没电了
	if (m_ChassisSensorValues.spCap.m_CapacitorVoltage < SPCAP_WARN_VOLTAGE)
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
	//SendCurrentToMotor();
}

void Chassis::ChassisAxisSpeedSet()
{
	//[TODO] 检验三角函数的符号
	if (m_CurChassisMode == DISABLE)
		m_VxSet = m_VySet = m_WzSet = 0;
	else if (m_CurChassisMode == FOLLOW_CHASSIS_YAW)
	{
		RCToChassisSpeed();
		m_AngleSet = ClampLoop(m_AngleSet - m_ChassisSensorValues.rc.ch[CHASSIS_Z_CHANNEL] * CHASSIS_WZ_RC_SEN, -M_PI, M_PI);
		double deltaAngle = ClampLoop(m_AngleSet - m_ChassisSensorValues.gyroZ, -M_PI, M_PI);
		m_WzSet = m_PIDChassisAngle.Calc(deltaAngle, 0, std::chrono::high_resolution_clock::now(), true); //符号为负？
		m_VxSet = Clamp(m_VxSet, -CHASSIS_VX_MAX, CHASSIS_VX_MAX);
		m_VySet = Clamp(m_VySet, -CHASSIS_VY_MAX, CHASSIS_VY_MAX);
	}
	else if (m_CurChassisMode == TOP)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.relativeAngle), sine = sin(m_ChassisSensorValues.relativeAngle);
		double vx = m_VxSet * cosine - m_VySet * sine;
		double vy = m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -CHASSIS_VX_MAX, CHASSIS_VX_MAX);
		m_VySet = Clamp(vy, -CHASSIS_VY_MAX, CHASSIS_VY_MAX);
		m_WzSet = Top_Wz;
	}
	else if (m_CurChassisMode == FOLLOW_GIMBAL_YAW)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.relativeAngle), sine = sin(m_ChassisSensorValues.relativeAngle);
		double vx = m_VxSet * cosine - m_VySet * sine;
		double vy = m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -CHASSIS_VX_MAX, CHASSIS_VX_MAX); 		
		m_VySet = Clamp(vy, -CHASSIS_VY_MAX, CHASSIS_VY_MAX);
		m_WzSet = m_PIDChassisAngle.Calc(m_ChassisSensorValues.relativeAngle, 0, std::chrono::high_resolution_clock::now(), true); //符号为负？
	}
	else if (m_CurChassisMode == OPENLOOP_Z)
	{
		RCToChassisSpeed();
		m_VxSet = Clamp(m_VxSet, -CHASSIS_VX_MAX, CHASSIS_VX_MAX);
		m_VySet = Clamp(m_VxSet, -CHASSIS_VY_MAX, CHASSIS_VY_MAX);
	}
}
