#ifndef OSSIAN_PID_HPP
#define OSSIAN_PID_HPP

#include <limits>
#include <cmath>

class PIDController
{
public:
	PIDController() : m_Kp(0.0), m_Ki(0.0), m_Kd(0.0)
	{
		m_Integral = m_LastError = 0.0;
		m_ThresError1 = m_ThresError2 = std::numeric_limits<double>::max();
		m_ThresIntegral = m_ThresOutput = std::numeric_limits<double>::max();
		m_DeadValue = 0.0;
		m_LastTimestamp = -1;
	}
	//����PID������
	void SetPIDParams(double kp, double ki, double kd)
	{
		m_Kp = kp;
		m_Ki = ki;
		m_Kd = kd;
	}

	//���ÿ�������ֵ
	void SetExpectation(double expectation)
	{
		m_Expectation = expectation;
	}

	void SetThresError(double th1, double th2)
	{
		m_ThresError1 = th1;
		m_ThresError2 = th2;
	}

	void SetThresIntegral(double th)
	{
		m_ThresIntegral = th;
	}

	void SetThresOutput(double th)
	{
		m_ThresOutput = th;
	}

	double Calc(double feedBack, long long curTimestamp)
	{
		long long interval = m_LastTimestamp > 0 ? curTimestamp - m_LastTimestamp : 1;
		m_LastTimestamp = curTimestamp;
		double ki = m_Ki * interval;
		double kd = m_Kd / interval;

		std::cout << "[exp] " << m_Expectation << " [fbk] " << feedBack << std::endl;
		double error = m_Expectation - feedBack;
		double output = 0.0;
		//���ַ��뷨��������� ��m_ThresError1 ��Χ�����������
		if (fabs(error) >= m_ThresError1)
			m_Integral = 0.0;
		//�������� ��m_ThresError2 ��Χ�ڣ��������֣�����������
		if (fabs(error) < m_ThresError2)
			m_Integral += error;
		//���������޷�����
		m_Integral = Clamp(m_Integral, m_ThresIntegral);

		output = m_Kp * error + ki * m_Integral + kd * (error - m_LastError);
		if (output > 0)
			output += m_DeadValue;
		else if (output < 0)
			output += -m_DeadValue;
		//��pid������޷�����
		return Clamp(output, m_ThresOutput);
	}

private:
	double m_Kp, m_Ki, m_Kd;
	long long m_LastTimestamp; //����PID����һ�����ĵ�ʱ���
	double m_LastError, m_Expectation, m_Integral;
	double m_ThresError1, m_ThresError2;  //ThresError1 > ThresError2
	double m_ThresIntegral, m_ThresOutput;
	double m_DeadValue; //������PID���С�ڴ�ֵ��ִ�л���û��Ӧ

};

#endif 