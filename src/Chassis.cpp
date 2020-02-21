
#include "Chassis.hpp"
#include "CtrlAlgorithms.hpp"


void Chassis::CalcWheelSpeed()
{
	Eigen::Vector3d vSet(m_VxSet, m_VySet, m_WzSet);
	m_WheelSpeedSet = m_WheelKinematicMat * vSet / WHEEL_RADIUS; //[4, 3] * [3, 1] -- > [4, 1]

	//������������ٶ�
	double maxWheelSpeedItem = m_WheelSpeedSet.maxCoeff();
	if (maxWheelSpeedItem > LIMIT_WHEEL_SPEED)
	{
		double scaleWheelSpeed = LIMIT_WHEEL_SPEED / maxWheelSpeedItem;
		m_WheelSpeedSet *= scaleWheelSpeed;
	}
}

void Chassis::ChassisPowerCtrl()
{
	//double curPwr, curBuf, maxPwr, maxBuf; //���ĸ����Ӳ���ϵͳ��ȡ
	double warnBuf = m_ChassisSensorValues.refereeMaxBuf * 0.9;
	double warnPwr = m_ChassisSensorValues.refereeMaxPwr * 0.9;
	
	double scalePwr;  //������Сϵ��
	double limitCurrent; //���̵���ܵ�������
	//��������С��Ԥ��ֵ��˵�����ʳ���
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
	//[TODO] ���̲���

	//һ�׵�ͨ�˲�����б�º�����Ϊ�����ٶ�����
	FirstOrderFilter foFilterVX(0.17), foFilterVY(0.33);
	m_VxSet = foFilterVX.Calc(vxChannelSet);
	m_VySet = foFilterVY.Calc(vyChannelSet);
}

void Chassis::ChassisModeSet()
{
	switch (m_ChassisSensorValues.rc.s[CHASSIS_MODE_CHANNEL])
	{
	case RC_SW_UP:
		m_CurChassisMode = FOLLOW_GIMBAL_YAW; break;
	case RC_SW_MID:
		m_CurChassisMode = TOP; break;
	case RC_SW_DOWN:
		m_CurChassisMode = FOLLOW_CHASSIS_YAW; break;
	default:
		m_CurChassisMode = DISABLE; break;
	}
	
}

void Chassis::ChassisCtrl()
{
	CalcWheelSpeed();
	for (size_t i = 0; i < 4; ++i)
		m_CurrentSend[i] = m_PIDChassisSpeed[i].Calc(m_WheelSpeedSet(i), m_Motors[i]->Status().m_RPM * MOTOR_RPM_TO_WHEEL_SPEED, m_Motors[i]->TimeStamp());
	
	//����������ݿ�û����
	if (m_ChassisSensorValues.spCapCurVtg < SPCAP_WARN_VOLTAGE)
		ChassisPowerCtrl();

	//SendCurrentToMotor();
}

void Chassis::ChassisAxisSpeedSet()
{
	//[TODO] �������Ǻ����ķ���
	if (m_CurChassisMode == DISABLE)
		m_VxSet = m_VySet = m_WzSet = 0;
	else if (m_CurChassisMode == FOLLOW_CHASSIS_YAW)
	{
		RCToChassisSpeed();
		m_AngleSet = ClampLoop(m_AngleSet - m_ChassisSensorValues.rc.ch[CHASSIS_Z_CHANNEL] * CHASSIS_WZ_RC_SEN, -PI, PI);
		double deltaAngle = ClampLoop(m_AngleSet - m_ChassisSensorValues.gyroZ, -PI, PI);
		m_WzSet = m_PIDChassisAngle.Calc(deltaAngle, 0, std::chrono::high_resolution_clock::now(), true); //����Ϊ����
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
		m_WzSet = TOP_WZ;
	}
	else if (m_CurChassisMode == FOLLOW_GIMBAL_YAW)
	{
		RCToChassisSpeed();
		double cosine = cos(m_ChassisSensorValues.relativeAngle), sine = sin(m_ChassisSensorValues.relativeAngle);
		double vx = m_VxSet * cosine - m_VySet * sine;
		double vy = m_VxSet * sine + m_VySet * cosine;
		m_VxSet = Clamp(vx, -CHASSIS_VX_MAX, CHASSIS_VX_MAX); 		
		m_VySet = Clamp(vy, -CHASSIS_VY_MAX, CHASSIS_VY_MAX);
		m_WzSet = m_PIDChassisAngle.Calc(m_ChassisSensorValues.relativeAngle, 0, std::chrono::high_resolution_clock::now(), true); //����Ϊ����
	}
}
