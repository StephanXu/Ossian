
#include "Gimbal.hpp"

std::array<double, 5> Gimbal::PIDAngleEcdPitchParams;
std::array<double, 5> Gimbal::PIDAngleGyroPitchParams;
std::array<double, 5> Gimbal::PIDAngleSpeedPitchParams;
std::array<double, 5> Gimbal::PIDAngleEcdYawParams;
std::array<double, 5> Gimbal::PIDAngleGyroYawParams;
std::array<double, 5> Gimbal::PIDAngleSpeedYawParams;



void Gimbal::GimbalCtrlSrcSet()
{
	switch (m_GimbalSensorValues.rc.sw[kGimbalModeChannel])
	{
	case kRCSwUp:
		m_GimbalCtrlSrc = RC; break;  
	case kRCSwMid:
		m_GimbalCtrlSrc = RC; break; //Aimbot
	case kRCSwDown:
		m_GimbalCtrlSrc = Disable; break;  //Mouse
	default:
		m_GimbalCtrlSrc = Disable; break;
	}

}

void Gimbal::GimbalCtrlInputProc()
{
	if (m_GimbalCtrlSrc == RC)
	{
		m_AngleInput[Pitch] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kPitchChannel], kGimbalRCDeadband) * kPitchRCSen; //遥控器传来的pitch角度期望rad
		m_AngleInput[Yaw] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kYawChannel], kGimbalRCDeadband) * kYawRCSen; //遥控器传来的yaw角度期望rad
	}
}


//遥控器：绝对量控制  [TODO]鼠标：增量控制
void Gimbal::GimbalExpAngleSet(MotorPosition position)
{
	double curEcdAngle = RelativeEcdToRad(m_Motors[position]->Status().m_Encoding, position == Pitch ? kPitchMidEcd : kYawMidEcd);
	if (m_GimbalCtrlSrc == Disable)
		return;
	else if (m_GimbalCtrlSrc == RC)
	{
		double ecdAngleAdd = m_AngleInput[position]; 
		if (m_CurGimbalAngleMode == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.imu.m_Pitch : m_GimbalSensorValues.imu.m_Yaw); 
			double errorAngle = ClampLoop(m_GyroAngleSet[position] - gyro, -M_PI, M_PI); 
			//判断会不会越过限位
			if (curEcdAngle + errorAngle + ecdAngleAdd > kMaxRelativeAngle[position])
			{
				if (ecdAngleAdd > 0)
					ecdAngleAdd = kMaxRelativeAngle[position] - errorAngle - curEcdAngle;
			}
			else if (curEcdAngle + errorAngle + ecdAngleAdd < kMinRelativeAngle[position])
			{
				if (ecdAngleAdd < 0)
					ecdAngleAdd = kMinRelativeAngle[position] - errorAngle - curEcdAngle;
			}
			m_GyroAngleSet[position] = ClampLoop(m_GyroAngleSet[position] + ecdAngleAdd, -M_PI, M_PI);
		}
		else if (m_CurGimbalAngleMode == Encoding)
		{
			m_EcdAngleSet[position] += ecdAngleAdd;
			m_EcdAngleSet[position] = Clamp(m_EcdAngleSet[position], kMinRelativeAngle[position], kMaxRelativeAngle[position]);
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
			double gyro = (position == Pitch ? m_GimbalSensorValues.imu.m_Pitch : m_GimbalSensorValues.imu.m_Yaw);
			double gyroSpeed = (position == Pitch ? m_GimbalSensorValues.imu.m_Wy : m_GimbalSensorValues.imu.m_Wz);
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
			double curEcdAngle = RelativeEcdToRad(m_Motors[position]->Status().m_Encoding, position == Pitch ? kPitchMidEcd : kYawMidEcd);
			double angleSpeedEcd = ClampLoop(curEcdAngle - m_LastEcdAngle[position], -M_PI, M_PI) / interval / 1000; //rad/s
			double angleSpeedSet = m_PIDAngleEcd[position].Calc(m_EcdAngleSet[position], curEcdAngle, std::chrono::high_resolution_clock::now(), true);
			m_CurrentSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, angleSpeedEcd, std::chrono::high_resolution_clock::now());

			m_LastEcdTimeStamp[position] = m_Motors[position]->TimeStamp();
			m_LastEcdAngle[position] = curEcdAngle;
		}
	}
}
