
#include "Gimbal.hpp"

std::array<double, 5> GimbalCtrlTask::PIDAngleEcdPitchParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleGyroPitchParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleSpeedPitchParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleEcdYawParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleGyroYawParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleSpeedYawParams;



void GimbalCtrlTask::GimbalCtrlSrcSet()
{
	SPDLOG_INFO("@FlagInitGimbal=[$flag={}]", static_cast<int>(m_FlagInitGimbal));
	if (m_FlagInitGimbal)
	{
		if (fabs(m_GimbalSensorValues.relativeAngle[Pitch]) < 0.1 
			&& fabs(m_GimbalSensorValues.relativeAngle[Yaw]) < 0.1)
		{
			SPDLOG_TRACE("Gimbal Init Done.");
			m_CurGimbalAngleMode = Encoding;
			m_EcdAngleSet[Pitch] = 0;
			m_EcdAngleSet[Yaw] = 0;
			m_GyroAngleSet[Pitch] = m_GimbalSensorValues.imu.m_Pitch;
			m_GyroAngleSet[Yaw] = m_GimbalSensorValues.imu.m_Yaw;

			/*m_LastEcdTimeStamp.fill(std::chrono::high_resolution_clock::time_point());
			m_PIDAngleEcd[Pitch].Reset();
			m_PIDAngleGyro[Pitch].Reset();
			m_PIDAngleSpeed[Pitch].Reset();

			m_PIDAngleEcd[Yaw].Reset();
			m_PIDAngleGyro[Yaw].Reset();
			m_PIDAngleSpeed[Yaw].Reset();*/

			m_FlagInitGimbal = false;
		}
		else
		{
			m_GimbalCtrlSrc = Init;
			if (m_GimbalSensorValues.rc.sw[kGimbalModeChannel] != kRCSwUp)
				m_GimbalCtrlSrc = Disable;
			m_CurGimbalAngleMode = Encoding;
			return;
		}
	}
	
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

void GimbalCtrlTask::GimbalCtrlInputProc()
{
	if (m_GimbalCtrlSrc == RC)
	{
		//遥控器传来的角度期望rad
		m_AngleInput[Pitch] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kPitchChannel], kGimbalRCDeadband) * kPitchRCSen; 
		m_AngleInput[Yaw] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kYawChannel], kGimbalRCDeadband) * kYawRCSen; 
		SPDLOG_INFO("@AngleInput=[$p={},$y={}]", m_AngleInput[Pitch], m_AngleInput[Yaw]);
	}
}


//遥控器：绝对量控制  [TODO]鼠标：增量控制
void GimbalCtrlTask::GimbalExpAngleSet(MotorPosition position)
{
	double curEcdAngle = m_GimbalSensorValues.relativeAngle[position];
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
			SPDLOG_INFO("@RelativeAngleYaw=[$min={},$max={}]", kMinRelativeAngle[Yaw], kMaxRelativeAngle[Yaw]);
		}
	}
	else if (m_GimbalCtrlSrc == Init)
	{
		m_EcdAngleSet[Pitch] = 0;
		m_EcdAngleSet[Yaw] = 0;
	}
	
}

void GimbalCtrlTask::GimbalCtrl(MotorPosition position)
{
	if (m_GimbalCtrlSrc == Disable)
		m_VoltageSend.fill(0);
	else if (m_GimbalCtrlSrc == RC || m_GimbalCtrlSrc == Init)
	{
		if (m_CurGimbalAngleMode == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.imu.m_Pitch : m_GimbalSensorValues.imu.m_Yaw);
			double gyroSpeed = (position == Pitch ? m_GimbalSensorValues.imu.m_Wy : m_GimbalSensorValues.imu.m_Wz);
			double angleSpeedSet = m_PIDAngleGyro[position].Calc(m_GyroAngleSet[position], gyro);
			m_VoltageSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, gyroSpeed);

			SPDLOG_INFO("@pidAngleGyro{}=[$SetAG{}={},$GetAG{}={}]",
				position,
				position,
				m_GyroAngleSet[position],
				position,
				gyro);
			SPDLOG_INFO("@pidAngleSpeed{}=[$SetAS{}={},$GetAS{}={}]",
				position,
				position,
				angleSpeedSet,
				position,
				gyroSpeed);
		}
		else if (m_CurGimbalAngleMode == Encoding)
		{
			//[TODO]用电机转速rpm换算出云台角速度
			double curEcdAngle = m_GimbalSensorValues.relativeAngle[position];
			/*//初始时刻，无法通过差分计算出角速度 
			if (m_LastEcdTimeStamp[position].time_since_epoch().count() == 0)
			{
				m_LastEcdTimeStamp[position] = m_Motors[position]->TimeStamp();
				m_LastEcdAngle[position] = curEcdAngle;
				return;
			}
			double interval = std::chrono::duration_cast<std::chrono::microseconds>(m_Motors[position]->TimeStamp() -
				m_LastEcdTimeStamp[position]).count() / 1000000.0;
			
			double angleSpeedEcd = ClampLoop(curEcdAngle - m_LastEcdAngle[position], -M_PI, M_PI) / interval; //rad/s*/
			//double angleSpeedSet = m_PIDAngleEcd[position].Calc(m_EcdAngleSet[position], curEcdAngle);

			//[TODO] 检查maxminRelativeAngle[Yaw]是否正确 --- 角度环set抖动
			double angleSpeedSet;
			if (position == Yaw)
			{
				angleSpeedSet = 3;
				
			}
			else if (position == Pitch)
			{
				angleSpeedSet = m_PIDAngleEcd[position].Calc(m_EcdAngleSet[position], curEcdAngle);
			}
			double gyroSpeed = (position == Pitch ? m_GimbalSensorValues.imu.m_Wy : m_GimbalSensorValues.imu.m_Wz);
			m_VoltageSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, gyroSpeed);

			/*m_LastEcdTimeStamp[position] = m_Motors[position]->TimeStamp();
			m_LastEcdAngle[position] = curEcdAngle;*/

			SPDLOG_INFO("@pidAngleEcd{}=[$SetAE{}={},$GetAE{}={}]",
				position, 
				position,
				m_EcdAngleSet[position],
				position,
				curEcdAngle);
			SPDLOG_INFO("@pidAngleSpeed{}=[$SetAS{}={},$GetAS{}={},$pidoutAS{}={}]",
				position,
				position,
				angleSpeedSet,
				position,
				gyroSpeed,
				position,
				m_VoltageSend[position]/10000.0);

		}
	}

	m_Gimbal->SendVoltageToMotors(m_VoltageSend);
}
