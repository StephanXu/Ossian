
#include "Gimbal.hpp"

std::array<double, 5> GimbalCtrlTask::PIDAngleEcdPitchParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleGyroPitchParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleGyroPitchAutoAimParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleSpeedPitchParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleEcdYawParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleGyroYawParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleGyroYawAutoAimParams;
std::array<double, 5> GimbalCtrlTask::PIDAngleSpeedYawParams;
std::array<double, 5> GimbalCtrlTask::PIDAutoAimInputParams;

//double GimbalCtrlTask::kAngleSpeedFilterCoef = 0;

void GimbalCtrlTask::GimbalCtrlModeSet()
{
	if (m_FlagInitGimbal)
	{
		static bool firstClosing = true;
		if (fabs(m_GimbalSensorValues.relativeAngle[Pitch]) < 0.05 
			&& fabs(m_GimbalSensorValues.relativeAngle[Yaw]) < 0.05)
		{
			if (firstClosing)
			{
				m_TimestampInit = std::chrono::high_resolution_clock::now();
				firstClosing = false;
			}

			long long interval = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - m_TimestampInit).count();
			//std::cerr << interval << std::endl;
			if (interval > 2000 && m_TimestampInit!= std::chrono::high_resolution_clock::time_point())  //在中值处稳定一段时间
			{
				std::cerr << "Gimbal Init Done!" << std::endl;
				m_CurGimbalAngleMode[Pitch] = Gyro;
				m_CurGimbalAngleMode[Yaw] = Gyro;
				m_EcdAngleSet[Pitch] = 0;
				m_EcdAngleSet[Yaw] = 0;
				m_GyroAngleSet[Pitch] = m_GimbalSensorValues.imu.m_Pitch;
				m_GyroAngleSet[Yaw] = m_GimbalSensorValues.imu.m_Yaw;

				std::for_each(m_EcdAngleFilters.begin(), m_EcdAngleFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });
				std::for_each(m_AngleSpeedFilters.begin(), m_AngleSpeedFilters.end(), [](FirstOrderFilter& x) { x.Reset(); });
				firstClosing = true;
				m_TimestampInit = std::chrono::high_resolution_clock::time_point();
				/*m_LastEcdTimeStamp.fill(std::chrono::high_resolution_clock::time_point());
				m_PIDAngleEcd[Pitch].Reset();
				m_PIDAngleGyro[Pitch].Reset();
				m_PIDAngleGyroAutoAim[Pitch].Reset();
				m_PIDAngleSpeed[Pitch].Reset();

				m_PIDAngleEcd[Yaw].Reset();
				m_PIDAngleGyro[Yaw].Reset();
				m_PIDAngleGyroAutoAim[Yaw].Reset();
				m_PIDAngleSpeed[Yaw].Reset();*/
				
				m_FlagInitGimbal = false;
			}
			else
			{
				m_GimbalCtrlMode = GimbalCtrlMode::Init;
				if (m_GimbalSensorValues.rc.sw[kGimbalModeChannel] != kRCSwMid
					&& m_GimbalSensorValues.rc.sw[kGimbalModeChannel] != kRCSwUp)  //右侧开关保持居中或上，云台归中，否则失能
					m_GimbalCtrlMode = GimbalCtrlMode::Disable;
				m_CurGimbalAngleMode.fill(Encoding);
				
				return;
			}
				
		}
		else
		{
			m_GimbalCtrlMode = GimbalCtrlMode::Init;
			if (m_GimbalSensorValues.rc.sw[kGimbalModeChannel] != kRCSwMid
				&& m_GimbalSensorValues.rc.sw[kGimbalModeChannel] != kRCSwUp)  //右侧开关保持居中，云台归中，否则失能
				m_GimbalCtrlMode = GimbalCtrlMode::Disable;
			m_CurGimbalAngleMode.fill(Encoding);
			
			//m_TimestampInit = std::chrono::high_resolution_clock::time_point();

			return;
		}

	}

	if (m_GimbalSensorValues.keyboardMode)
	{
		//键鼠，长按右键自瞄
		if (m_GimbalSensorValues.rc.click[1])
			m_GimbalCtrlMode = GimbalCtrlMode::Aimbot;
		else
			m_GimbalCtrlMode = GimbalCtrlMode::RC;

		if (m_GimbalSensorValues.rc.sw[kGimbalModeChannel] == kRCSwDown)
			m_GimbalCtrlMode = GimbalCtrlMode::Disable;
	}
	else
	{
		//遥控器
		switch (m_GimbalSensorValues.rc.sw[kGimbalModeChannel])
		{
		case kRCSwUp:
			m_GimbalCtrlMode = GimbalCtrlMode::Aimbot; break;
		case kRCSwMid:
			m_GimbalCtrlMode = GimbalCtrlMode::RC; break; //Aimbot
		case kRCSwDown:
			m_GimbalCtrlMode = GimbalCtrlMode::Disable; break;  //Mouse
		default:
			m_GimbalCtrlMode = GimbalCtrlMode::Disable; break;
		}
	}
	

	
}

void GimbalCtrlTask::GimbalCtrlInputProc()
{
	if (m_GimbalCtrlMode == GimbalCtrlMode::RC)
	{
		if (m_GimbalSensorValues.keyboardMode)
		{
			//键鼠
			m_AngleInput[Pitch] = m_GimbalSensorValues.rc.mouse[1] * kPitchMouseSen;
			m_AngleInput[Yaw] = m_GimbalSensorValues.rc.mouse[0] * kYawMouseSen;
		}
		else
		{
			//遥控器
			//遥控器传来的角度期望rad
			m_AngleInput[Pitch] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kPitchChannel], kGimbalRCDeadband) * kPitchRCSen;
			m_AngleInput[Yaw] = DeadbandLimit(m_GimbalSensorValues.rc.ch[kYawChannel], kGimbalRCDeadband) * kYawRCSen;
		}
		
		//SPDLOG_TRACE("@AngleInput=[$p={},$y={}]", m_AngleInput[Pitch], m_AngleInput[Yaw]);
		//std::cerr << "AngleInput: " << m_AngleInput[Pitch] << '\t' << m_AngleInput[Yaw] << std::endl;
	}
	else if (m_GimbalCtrlMode == GimbalCtrlMode::Aimbot)
	{ 
		//auto filterdAngles = m_AutoAimPredictor.Predict();
		//std::cerr << filterdAngles << std::endl;
		//m_AutoAimPredictor.Correct(m_GimbalSensorValues.autoAimStatus.m_Pitch, m_GimbalSensorValues.autoAimStatus.m_Yaw);

		m_AngleInput[Pitch] = m_PIDAutoAimInput[Pitch].Calc(m_GimbalSensorValues.autoAimStatus.m_Pitch, 0);
		m_AngleInput[Yaw] = m_PIDAutoAimInput[Yaw].Calc(m_GimbalSensorValues.autoAimStatus.m_Yaw, 0);

		/*m_AngleInput[Pitch] = m_PIDAutoAimInput[Pitch].Calc(m_GimbalSensorValues.autoAimStatus.m_Pitch, 0);
		m_AngleInput[Yaw] = m_PIDAutoAimInput[Yaw].Calc(m_GimbalSensorValues.autoAimStatus.m_Yaw, 0);*/
		//std::cerr << "AimbotInput: " << m_AngleInput[Pitch] << '\t' << m_AngleInput[Yaw] << std::endl;
	}
}


//遥控器：绝对量控制  [TODO]鼠标：增量控制
void GimbalCtrlTask::GimbalExpAngleSet(MotorPosition position)
{
	double curEcdAngle = m_GimbalSensorValues.relativeAngle[position];
	if (m_GimbalCtrlMode == GimbalCtrlMode::Disable)
		return;
	else if (m_GimbalCtrlMode == GimbalCtrlMode::RC || m_GimbalCtrlMode == GimbalCtrlMode::Aimbot)
	{
		double angleInput = m_AngleInput[position]; 
		if (m_CurGimbalAngleMode[position] == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.imu.m_Pitch : m_GimbalSensorValues.imu.m_Yaw); 
			double errorAngle = ClampLoop(m_GyroAngleSet[position] - gyro, -M_PI, M_PI); 
			//只判断pitch会不会越过限位，yaw是360度旋转
			if (position == Pitch)
			{
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
			}
			
			//std::cerr << angleInput << '\t' << kMinRelativeAngle[position] << '\t' << kMaxRelativeAngle[position] << std::endl;
			m_GyroAngleSet[position] = ClampLoop(m_GyroAngleSet[position] + angleInput, -M_PI, M_PI);
		}
		else if (m_CurGimbalAngleMode[position] == Encoding)
		{
			m_EcdAngleSet[position] += angleInput;
			m_EcdAngleSet[position] = Clamp(m_EcdAngleSet[position], kMinRelativeAngle[position], 
											kMaxRelativeAngle[position]);
			//SPDLOG_TRACE("@RelativeAngleYaw=[$min={},$max={}]", kMinRelativeAngle[Yaw], kMaxRelativeAngle[Yaw]);
		}
	}
	else if (m_GimbalCtrlMode == GimbalCtrlMode::Init)
	{
		m_EcdAngleSet[Pitch] = 0;
		m_EcdAngleSet[Yaw] = 0;
	}
	
}

void GimbalCtrlTask::GimbalCtrl(MotorPosition position)
{
	if (m_GimbalCtrlMode == GimbalCtrlMode::Disable)
		m_VoltageSend.fill(0);
	else if (m_GimbalCtrlMode == GimbalCtrlMode::RC || m_GimbalCtrlMode == GimbalCtrlMode::Init || m_GimbalCtrlMode == GimbalCtrlMode::Aimbot)
	{
		double angleSpeedSet = 0;
		double gyroSpeed = (position == Pitch ? m_GimbalSensorValues.imu.m_YAngleSpeed : m_GimbalSensorValues.imu.m_ZAngleSpeed);
		//////double filteredRPM = m_RPMFilters[position].Calc(m_MotorsStatus.m_RPM[position]);
		//////gyroSpeed = m_AngleSpeedFilters[position].Calc(gyroSpeed);
		if (m_CurGimbalAngleMode[position] == Gyro)
		{
			double gyro = (position == Pitch ? m_GimbalSensorValues.imu.m_Pitch : m_GimbalSensorValues.imu.m_Yaw);
			
			if(m_GimbalCtrlMode == GimbalCtrlMode::Aimbot || m_GimbalCtrlMode == GimbalCtrlMode::Windmill)
				angleSpeedSet = m_PIDAngleGyroAutoAim[position].Calc(m_GyroAngleSet[position], gyro);
			else
				angleSpeedSet = m_PIDAngleGyro[position].Calc(m_GyroAngleSet[position], gyro);
			SPDLOG_TRACE("@pidAngleGyro{}=[$SetAG{}={},$GetAG{}={},$pidoutAG{}={}]",
				position,
				position,
				m_GyroAngleSet[position],
				position,
				gyro,
				position,
				angleSpeedSet);
		}
		else if (m_CurGimbalAngleMode[position] == Encoding)
		{
			//[TODO]用电机转速rpm换算出云台角速度
			double curEcdAngle = m_EcdAngleFilters[position].Calc(m_GimbalSensorValues.relativeAngle[position]);
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
			angleSpeedSet = m_PIDAngleEcd[position].Calc(m_EcdAngleSet[position], curEcdAngle);
			/*SPDLOG_TRACE("@pidAngleEcd{}=[$SetAE{}={},$GetAE{}={},$pidoutAE{}={}]",
			position,
			position,
			m_EcdAngleSet[position],
			position,
			curEcdAngle,
			position,
			angleSpeedSet);*/
			//[TODO] 尝试将速度环的set与get都扩大相同的倍数，便于调参
		}

		m_VoltageSend[position] = m_PIDAngleSpeed[position].Calc(angleSpeedSet, gyroSpeed);
		
		/*SPDLOG_TRACE("@pidAngleSpeed{}=[$SetAS{}={},$GetAS{}={}]",
			position,
			position,
			angleSpeedSet,
			position,
			gyroSpeed);*/
	}
	m_Gimbal->SendVoltageToMotors(m_VoltageSend);
}
