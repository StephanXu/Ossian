
#include "Gimbal.hpp"


void Gimbal::GimbalCtrlModeSet()
{
	switch (m_GimbalSensorValues.rc.sw[GIMBAL_MODE_CHANNEL])
	{
	case RC_SW_UP:
		m_CtrlMode = RC; break;
	case RC_SW_MID:
		m_CtrlMode = AUTOAIM; break;
	case RC_SW_DOWN:
		m_CtrlMode = MOUSE; break;
	default:
		m_CtrlMode = DISABLE; break;
	}

}

double Gimbal::PitchCtrlInputProc()
{
	if (m_CtrlMode == RC)
	{
		return DeadbandLimit(m_GimbalSensorValues.rc.ch[PITCH_CHANNEL], GIMBAL_RC_DEADBAND) * PITCH_RC_SEN; //遥控器传来的pitch角度期望rad
	}

}

double Gimbal::YawCtrlInputProc()
{
	if (m_CtrlMode == RC)
	{
		return DeadbandLimit(m_GimbalSensorValues.rc.ch[YAW_CHANNEL], GIMBAL_RC_DEADBAND) * YAW_RC_SEN; //遥控器传来的yaw角度期望rad
	}
}

//遥控器：绝对量控制  [TODO]鼠标：增量控制
void Gimbal::SetPitch(double angleInput, uint16_t curEcd)
{
	double curEcdAngle = curEcd * MOTOR_ECD_TO_RAD_COEF;
	if (m_CtrlMode == RC)
	{
		double ecdAngleAdd = angleInput - curEcdAngle;
		if (m_CurGimbalAngleMode == GYROANGLE)
		{
			double errorAngle = ClampLoop(m_GyroPitchAngleSet - m_GimbalSensorValues.gyroY, -M_PI, M_PI);
			//判断会不会越过机械限位
			if (curEcdAngle + errorAngle + ecdAngleAdd > PITCH_MAX_RELATIVE_ANGLE)
			{
				if (ecdAngleAdd > 0)
					ecdAngleAdd = PITCH_MAX_RELATIVE_ANGLE - errorAngle - curEcdAngle;
			}
			else if (curEcdAngle + errorAngle + ecdAngleAdd < PITCH_MIN_RELATIVE_ANGLE)
			{
				if (ecdAngleAdd < 0)
					ecdAngleAdd = PITCH_MIN_RELATIVE_ANGLE - errorAngle - curEcdAngle;
			}
			m_GyroPitchAngleSet = ClampLoop(m_GyroPitchAngleSet + ecdAngleAdd, -M_PI, M_PI);
		}
		else if (m_CurGimbalAngleMode == ECDANGLE)
		{
			m_EcdPitchAngleSet += ecdAngleAdd;
			m_EcdPitchAngleSet = Clamp(m_EcdPitchAngleSet, PITCH_MIN_RELATIVE_ANGLE, PITCH_MAX_RELATIVE_ANGLE);
		}
	}
	
}

//遥控器：绝对量控制  [TODO]鼠标：增量控制
void Gimbal::SetYaw(double angleInput, uint16_t curEcd)
{
	double curEcdAngle = curEcd * MOTOR_ECD_TO_RAD_COEF;
	if (m_CtrlMode == RC)
	{
		double ecdAngleAdd = angleInput - curEcdAngle;
		if (m_CurGimbalAngleMode == GYROANGLE)
		{
			double errorAngle = ClampLoop(m_GyroYawAngleSet - m_GimbalSensorValues.gyroZ, -M_PI, M_PI);
			//判断会不会越过机械限位
			if (curEcdAngle + errorAngle + ecdAngleAdd > YAW_MAX_RELATIVE_ANGLE)
			{
				if (ecdAngleAdd > 0)
					ecdAngleAdd = YAW_MAX_RELATIVE_ANGLE - errorAngle - curEcdAngle;
			}
			else if (curEcdAngle + errorAngle + ecdAngleAdd < YAW_MIN_RELATIVE_ANGLE)
			{
				if (ecdAngleAdd < 0)
					ecdAngleAdd = YAW_MIN_RELATIVE_ANGLE - errorAngle - curEcdAngle;
			}
			m_GyroYawAngleSet = ClampLoop(m_GyroYawAngleSet + ecdAngleAdd, -M_PI, M_PI);
		}
		else if (m_CurGimbalAngleMode == ECDANGLE)
		{
			m_EcdYawAngleSet += ecdAngleAdd;
			m_EcdYawAngleSet = Clamp(m_EcdYawAngleSet, YAW_MIN_RELATIVE_ANGLE, YAW_MAX_RELATIVE_ANGLE);
		}
	}
}

void Gimbal::CtrlPitch()
{
	double pitchCurrentSend = 0;
	if (m_CurGimbalAngleMode == GYROANGLE)
	{
		double angleSpeedSet = m_PIDPitchAngleGyro.Calc(m_GyroPitchAngleSet, m_GimbalSensorValues.gyroY, std::chrono::high_resolution_clock::now(), true);
		pitchCurrentSend = m_PIDPitchAngleSpeed.Calc(angleSpeedSet, m_GimbalSensorValues.gyroSpeedY, std::chrono::high_resolution_clock::now());
	}
	else if (m_CurGimbalAngleMode == ECDANGLE)
	{
		//[TODO]用电机转速rpm换算出云台角速度
		//初始时刻，无法通过差分计算出角速度 
		if (m_LastEcdTimeStampPitch.time_since_epoch().count() == 0)  
			return;
		double interval = std::chrono::duration<double, std::milli>(m_Motors[Pitch]->TimeStamp() - m_LastEcdTimeStampPitch).count();  //ms
		double curEcdAnglePitch = m_Motors[Pitch]->Status().m_Encoding * MOTOR_ECD_TO_RAD_COEF;
		double angleSpeedEcd = ClampLoop(curEcdAnglePitch - m_LastEcdAnglePitch, -M_PI, M_PI) / interval / 1000; //rad/s
		double angleSpeedSet = m_PIDPitchAngleEcd.Calc(m_EcdPitchAngleSet, curEcdAnglePitch, std::chrono::high_resolution_clock::now(), true);
		pitchCurrentSend = m_PIDPitchAngleSpeed.Calc(angleSpeedSet, angleSpeedEcd, std::chrono::high_resolution_clock::now());
		
		m_LastEcdTimeStampPitch = m_Motors[Pitch]->TimeStamp();
		m_LastEcdAnglePitch = curEcdAnglePitch;
	}
	//SendCurrentToMotorPitch(pitchCurrentSend);
}

void Gimbal::CtrlYaw()
{
	double yawCurrentSend = 0;
	if (m_CurGimbalAngleMode == GYROANGLE)
	{
		double angleSpeedSet = m_PIDYawAngleGyro.Calc(m_GyroYawAngleSet, m_GimbalSensorValues.gyroZ, std::chrono::high_resolution_clock::now(), true);
		yawCurrentSend = m_PIDYawAngleSpeed.Calc(angleSpeedSet, m_GimbalSensorValues.gyroSpeedZ, std::chrono::high_resolution_clock::now());
	}
	else if (m_CurGimbalAngleMode == ECDANGLE)
	{
		//[TODO]用电机转速rpm换算出云台角速度
		//初始时刻，无法通过差分计算出角速度 
		if (m_LastEcdTimeStampYaw.time_since_epoch().count() == 0)
			return;
		double interval = std::chrono::duration<double, std::milli>(m_Motors[Yaw]->TimeStamp() - m_LastEcdTimeStampYaw).count();  //ms
		double curEcdAngleYaw = m_Motors[Yaw]->Status().m_Encoding * MOTOR_ECD_TO_RAD_COEF;
		double angleSpeedEcd = ClampLoop(curEcdAngleYaw - m_LastEcdAngleYaw, -M_PI, M_PI) / interval / 1000; //rad/s
		double angleSpeedSet = m_PIDYawAngleEcd.Calc(m_EcdYawAngleSet, curEcdAngleYaw, std::chrono::high_resolution_clock::now(), true);
		yawCurrentSend = m_PIDYawAngleSpeed.Calc(angleSpeedSet, angleSpeedEcd, std::chrono::high_resolution_clock::now());

		m_LastEcdTimeStampYaw = m_Motors[Yaw]->TimeStamp();
		m_LastEcdAngleYaw = curEcdAngleYaw;
	}
	//SendCurrentToMotorYaw(yawCurrentSend);
}
