
#include "Gimbal.hpp"


void Gimbal::GimbalCtrlSrcSet()
{
	switch (m_GimbalSensorValues.rc.sw[GIMBAL_MODE_CHANNEL])
	{
	case RC_SW_UP:
		m_GimbalCtrlSrc = RC; break;
	case RC_SW_MID:
		m_GimbalCtrlSrc = Aimbot; break;
	case RC_SW_DOWN:
		m_GimbalCtrlSrc = Disable; break;  //MOUSE
	default:
		m_GimbalCtrlSrc = Disable; break;
	}

}

void Gimbal::GimbalCtrlInputProc()
{
	if (m_GimbalCtrlSrc == RC)
	{
		m_AngleInput[Pitch] = DeadbandLimit(m_GimbalSensorValues.rc.ch[PITCH_CHANNEL], GIMBAL_RC_DEADBAND) * PITCH_RC_SEN; //遥控器传来的pitch角度期望rad
		m_AngleInput[Yaw] = DeadbandLimit(m_GimbalSensorValues.rc.ch[YAW_CHANNEL], GIMBAL_RC_DEADBAND) * YAW_RC_SEN; //遥控器传来的yaw角度期望rad
	}
}


//遥控器：绝对量控制  [TODO]鼠标：增量控制
void Gimbal::GimbalExpAngleSet(MotorPosition position)
{
	double curEcdAngle = RelativeEcdToRad(m_Motors[position]->Status().m_Encoding, position == Pitch ? PITCH_MID_ECD : YAW_MID_ECD);
	if (m_GimbalCtrlSrc == Disable)
		return;
	if (m_GimbalCtrlSrc == RC)
	{
		double ecdAngleAdd = m_AngleInput[position] - curEcdAngle;
		if (m_CurGimbalAngleMode == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.gyroY : m_GimbalSensorValues.gyroZ);
			double errorAngle = ClampLoop(m_GyroAngleSet[position] - gyro, -M_PI, M_PI);
			//判断会不会越过限位
			if (curEcdAngle + errorAngle + ecdAngleAdd > MAX_RELATIVE_ANGLE[position])
			{
				if (ecdAngleAdd > 0)
					ecdAngleAdd = MAX_RELATIVE_ANGLE[position] - errorAngle - curEcdAngle;
			}
			else if (curEcdAngle + errorAngle + ecdAngleAdd < MIN_RELATIVE_ANGLE[position])
			{
				if (ecdAngleAdd < 0)
					ecdAngleAdd = MIN_RELATIVE_ANGLE[position] - errorAngle - curEcdAngle;
			}
			m_GyroAngleSet[position] = ClampLoop(m_GyroAngleSet[position] + ecdAngleAdd, -M_PI, M_PI);
		}
		else if (m_CurGimbalAngleMode == Encoding)
		{
			m_EcdAngleSet[position] += ecdAngleAdd;
			m_EcdAngleSet[position] = Clamp(m_EcdAngleSet[position], MIN_RELATIVE_ANGLE[position], MAX_RELATIVE_ANGLE[position]);
		}
	}
	
}

void Gimbal::GimbalCtrlCalc(MotorPosition position)
{
	if (m_GimbalCtrlSrc == Disable)
		m_CurrentSend.fill(0);
	else if (m_GimbalCtrlSrc == RC)
	{
		if (m_CurGimbalAngleMode == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.gyroY : m_GimbalSensorValues.gyroZ);
			double gyroSpeed = (position == Pitch ? m_GimbalSensorValues.gyroSpeedY : m_GimbalSensorValues.gyroSpeedZ);
			double angleSpeedSet = m_PIDAngleGyro[position].Calc(m_GyroAngleSet[position], gyro, std::chrono::high_resolution_clock::now(), true);
			m_CurrentSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, gyroSpeed, std::chrono::high_resolution_clock::now());
		}
		else if (m_CurGimbalAngleMode == Encoding)
		{
			//[TODO]用电机转速rpm换算出云台角速度
			//初始时刻，无法通过差分计算出角速度 
			if (m_LastEcdTimeStamp[position].time_since_epoch().count() == 0)
				return;
			double interval = std::chrono::duration<double, std::milli>(m_Motors[position]->TimeStamp() - m_LastEcdTimeStamp[position]).count();  //ms
			double curEcdAngle = RelativeEcdToRad(m_Motors[position]->Status().m_Encoding, position == Pitch ? PITCH_MID_ECD : YAW_MID_ECD);
			double angleSpeedEcd = ClampLoop(curEcdAngle - m_LastEcdAngle[position], -M_PI, M_PI) / interval / 1000; //rad/s
			double angleSpeedSet = m_PIDAngleEcd[position].Calc(m_EcdAngleSet[position], curEcdAngle, std::chrono::high_resolution_clock::now(), true);
			m_CurrentSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, angleSpeedEcd, std::chrono::high_resolution_clock::now());

			m_LastEcdTimeStamp[position] = m_Motors[position]->TimeStamp();
			m_LastEcdAngle[position] = curEcdAngle;
		}
	}
}
