
#include "Gimbal.hpp"


double Gimbal::PitchCtrlInputProc()
{
	if (m_GimbalSensorValues.rc.s[GIMBAL_MODE_CHANNEL] == RC_SW_DOWN)
		m_CtrlMode = MOUSE;
	else if (m_GimbalSensorValues.rc.s[GIMBAL_MODE_CHANNEL] == RC_SW_MID)
		m_CtrlMode = AUTOAIM;
	if (m_CurGimbalMode == RC)
	{
		if (m_GimbalSensorValues.rc.s[GIMBAL_MODE_CHANNEL] == RC_SW_UP) //����
			return PITCH_MID_RAD;
		else
			return DeadbandLimit(m_GimbalSensorValues.rc.ch[PITCH_CHANNEL], GIMBAL_RC_DEADBAND) * PITCH_RC_SEN; //ң����������pitch�Ƕ�����rad
	}

}

double Gimbal::YawCtrlInputProc()
{
	if (m_GimbalSensorValues.rc.s[GIMBAL_MODE_CHANNEL] == RC_SW_DOWN)
		m_CtrlMode = MOUSE;
	else if (m_GimbalSensorValues.rc.s[GIMBAL_MODE_CHANNEL] == RC_SW_MID)
		m_CtrlMode = AUTOAIM;
	if (m_CurGimbalMode == RC)
	{
		if (m_GimbalSensorValues.rc.s[GIMBAL_MODE_CHANNEL] == RC_SW_UP) //����
			return YAW_MID_RAD;
		else
			return DeadbandLimit(m_GimbalSensorValues.rc.ch[YAW_CHANNEL], GIMBAL_RC_DEADBAND) * YAW_RC_SEN; //ң����������yaw�Ƕ�����rad
	}
}

//ң����������������  [TODO]��꣺��������
void Gimbal::SetPitch(double angleInput, uint16_t curEcd)
{
	double curEcdAngle = curEcd * MOTOR_ECD_TO_RAD_COEF;
	if (m_CtrlMode == RC)
	{
		double ecdAngleAdd = angleInput - curEcdAngle;
		if (m_CurGimbalMode == GYROANGLE)
		{
			double errorAngle = ClampLoop(m_GyroPitchAngleSet - m_GimbalSensorValues.gyroY, -PI, PI);
			//�жϻ᲻��Խ����е��λ
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
			m_GyroPitchAngleSet = ClampLoop(m_GyroPitchAngleSet + ecdAngleAdd, -PI, PI);
		}
		else if (m_CurGimbalMode == ECDANGLE)
		{
			m_EcdPitchAngleSet += ecdAngleAdd;
			m_EcdPitchAngleSet = Clamp(m_EcdPitchAngleSet, PITCH_MIN_RELATIVE_ANGLE, PITCH_MAX_RELATIVE_ANGLE);
		}
	}
	
}

void Gimbal::SetYaw(double angleInput, uint16_t curEcd)
{
	double curEcdAngle = curEcd * MOTOR_ECD_TO_RAD_COEF;
	if (m_CtrlMode == RC)
	{
		double ecdAngleAdd = angleInput - curEcdAngle;
		if (m_CurGimbalMode == GYROANGLE)
		{
			double errorAngle = ClampLoop(m_GyroYawAngleSet - m_GimbalSensorValues.gyroZ, -PI, PI);
			//�жϻ᲻��Խ����е��λ
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
			m_GyroYawAngleSet = ClampLoop(m_GyroYawAngleSet + ecdAngleAdd, -PI, PI);
		}
		else if (m_CurGimbalMode == ECDANGLE)
		{
			m_EcdYawAngleSet += ecdAngleAdd;
			m_EcdYawAngleSet = Clamp(m_EcdYawAngleSet, YAW_MIN_RELATIVE_ANGLE, YAW_MAX_RELATIVE_ANGLE);
		}
	}
}

void Gimbal::CtrlPitch()
{
	double pitchCurrentSend = 0;
	if (m_CurGimbalMode == GYROANGLE)
	{
		double angleSpeedSet = m_PIDPitchAngleGyro.Calc(m_GyroPitchAngleSet, m_GimbalSensorValues.gyroY, std::chrono::high_resolution_clock::now(), true);
		pitchCurrentSend = m_PIDPitchAngleSpeed.Calc(angleSpeedSet, m_GimbalSensorValues.gyroSpeedY, std::chrono::high_resolution_clock::now());
	}
	else if (m_CurGimbalMode == ECDANGLE)
	{
		//[TODO]�õ��ת��rpm�������̨���ٶ�
		//��ʼʱ�̣��޷�ͨ����ּ�������ٶ� 
		if (m_LastEcdTimeStampPitch.time_since_epoch().count() == 0)  
			return;
		double interval = std::chrono::duration<double, std::milli>(m_Motors[Pitch]->TimeStamp() - m_LastEcdTimeStampPitch).count();  //ms
		double curEcdAnglePitch = m_Motors[Pitch]->Status().m_Encoding * MOTOR_ECD_TO_RAD_COEF;
		double angleSpeedEcd = ClampLoop(curEcdAnglePitch - m_LastEcdAnglePitch, -PI, PI) / interval / 1000; //rad/s
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
	if (m_CurGimbalMode == GYROANGLE)
	{
		double angleSpeedSet = m_PIDYawAngleGyro.Calc(m_GyroYawAngleSet, m_GimbalSensorValues.gyroZ, std::chrono::high_resolution_clock::now(), true);
		yawCurrentSend = m_PIDYawAngleSpeed.Calc(angleSpeedSet, m_GimbalSensorValues.gyroSpeedZ, std::chrono::high_resolution_clock::now());
	}
	else if (m_CurGimbalMode == ECDANGLE)
	{
		//[TODO]�õ��ת��rpm�������̨���ٶ�
		//��ʼʱ�̣��޷�ͨ����ּ�������ٶ� 
		if (m_LastEcdTimeStampYaw.time_since_epoch().count() == 0)
			return;
		double interval = std::chrono::duration<double, std::milli>(m_Motors[Yaw]->TimeStamp() - m_LastEcdTimeStampYaw).count();  //ms
		double curEcdAngleYaw = m_Motors[Yaw]->Status().m_Encoding * MOTOR_ECD_TO_RAD_COEF;
		double angleSpeedEcd = ClampLoop(curEcdAngleYaw - m_LastEcdAngleYaw, -PI, PI) / interval / 1000; //rad/s
		double angleSpeedSet = m_PIDYawAngleEcd.Calc(m_EcdYawAngleSet, curEcdAngleYaw, std::chrono::high_resolution_clock::now(), true);
		yawCurrentSend = m_PIDYawAngleSpeed.Calc(angleSpeedSet, angleSpeedEcd, std::chrono::high_resolution_clock::now());

		m_LastEcdTimeStampYaw = m_Motors[Yaw]->TimeStamp();
		m_LastEcdAngleYaw = curEcdAngleYaw;
	}
	//SendCurrentToMotorYaw(yawCurrentSend);
}
