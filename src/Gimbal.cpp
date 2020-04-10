
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
		m_GimbalCtrlSrc = Disable; break; //Aimbot
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
		//遥控器传来的角度期望rad
		m_AngleInput[Pitch] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kPitchChannel], kGimbalRCDeadband) * kPitchRCSen; 
		m_AngleInput[Yaw] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kYawChannel], kGimbalRCDeadband) * kYawRCSen; 
	}
}


//遥控器：绝对量控制  [TODO]鼠标：增量控制
void Gimbal::GimbalExpAngleSet(MotorPosition position)
{
	double curEcdAngle = RelativeEcdToRad(m_Motors[position]->Status().m_Encoding, position == Pitch ? kPitchMidEcd : 
		kYawMidEcd);
	if (m_GimbalCtrlSrc == Disable)
		return;
	else if (m_GimbalCtrlSrc == RC)
	{
		double angleInput = m_AngleInput[position]; 
		if (m_CurGimbalAngleMode == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.imu.m_Pitch : m_GimbalSensorValues.imu.m_Yaw); 
			double errorAngle = ClampLoop(m_GyroAngleSet[position] - gyro, -M_PI, M_PI); 
			//判断会不会越过限位
			if (curEcdAngle + errorAngle + angleInput > kMaxRelativeAngle[position])
			{
				if (angleInput > 0)
					angleInput = kMaxRelativeAngle[position] - errorAngle - curEcdAngle;
			}
			else if (curEcdAngle + errorAngle + angleInput < kMinRelativeAngle[position])
			{
				if (angleInput < 0)
					angleInput = kMinRelativeAngle[position] - errorAngle - curEcdAngle;
			}
			m_GyroAngleSet[position] = ClampLoop(m_GyroAngleSet[position] + angleInput, -M_PI, M_PI);
		}
		else if (m_CurGimbalAngleMode == Encoding)
		{
			m_EcdAngleSet[position] += angleInput;
			m_EcdAngleSet[position] = Clamp(m_EcdAngleSet[position], kMinRelativeAngle[position], 
											kMaxRelativeAngle[position]);
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
			double angleSpeedSet = m_PIDAngleGyro[position].Calc(m_GyroAngleSet[position], gyro, hrClock::now(), true);
			m_CurrentSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, gyroSpeed, hrClock::now());
		}
		else if (m_CurGimbalAngleMode == Encoding)
		{
			//[TODO]用电机转速rpm换算出云台角速度
			double curEcdAngle = RelativeEcdToRad(m_Motors[position]->Status().m_Encoding, position == Pitch ?
				kPitchMidEcd : kYawMidEcd);
			//初始时刻，无法通过差分计算出角速度 
			if (m_LastEcdTimeStamp[position].time_since_epoch().count() == 0)
			{
				m_LastEcdTimeStamp[position] = m_Motors[position]->TimeStamp();
				m_LastEcdAngle[position] = curEcdAngle;
				return;
			}
			double interval = std::chrono::duration<double, std::milli>(m_Motors[position]->TimeStamp() -
				m_LastEcdTimeStamp[position]).count() / 1000.0;  //s
			
			double angleSpeedEcd = ClampLoop(curEcdAngle - m_LastEcdAngle[position], -M_PI, M_PI) / interval; //rad/s
			double angleSpeedSet = m_PIDAngleEcd[position].Calc(m_EcdAngleSet[position], curEcdAngle, hrClock::now(), 
																true);
			m_CurrentSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, angleSpeedEcd, hrClock::now());

			m_LastEcdTimeStamp[position] = m_Motors[position]->TimeStamp();
			m_LastEcdAngle[position] = curEcdAngle;

			spdlog::info("@pidAngleEcd{}=[$set={},$get={},$pidout={}]", 
				position, 
				m_EcdAngleSet[position], 
				curEcdAngle, 
				angleSpeedSet);
			spdlog::info("@pidAngleSpeed{}=[$set={},$get={},$pidout={}]",
				position,
				angleSpeedSet,
				angleSpeedEcd,
				m_CurrentSend[position]);

		}
	}
}
