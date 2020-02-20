#include <cmath>
#include <limits>
#include <chrono>

constexpr double DOUBLE_MAX = std::numeric_limits<double>::max() - 5;
constexpr double PI = 3.14159265359; 
constexpr double MOTOR_ECD_TO_RAD_COEF = 2 * PI / 8192;
constexpr uint16_t HALF_ECD_RANGE = 4096;
constexpr uint16_t ECD_RANGE = 8191;



// �޷�����
template<typename T>
inline T Clamp(T value, const T lowerBnd, const T upperBnd)
{
	if (value > upperBnd)
		return upperBnd;
	else if (value < lowerBnd)
		return lowerBnd;
	else
		return value;
}

//��������
template<typename T>
inline T DeadbandLimit(T value, const T deadband)
{
	if (value > deadband || value < -deadband)
		return value;
	else
		return 0;
}

//ѭ���޷�
template<typename T>
inline T ClampLoop(T value, const T lowerBnd, const T upperBnd)
{
	T len = upperBnd - lowerBnd;
	if (value > upperBnd)
		while (value > upperBnd)
			value -= len;
	else if (value < lowerBnd)
		while (value < lowerBnd)
			value += len;
	return value;
}

//���㵱ǰ����ֵ����ֵ֮�����ԽǶ�rad
double RelativeEcdToRad(uint16_t ecd, const uint16_t ecdMid)
{
	int relativeEcd = ecd - ecdMid;
	if (relativeEcd > HALF_ECD_RANGE)
		relativeEcd -= ECD_RANGE;
	else if (relativeEcd < -HALF_ECD_RANGE)
		relativeEcd += ECD_RANGE;
	return relativeEcd * MOTOR_ECD_TO_RAD_COEF;
}

// һ�׵�ͨ�˲���
class FirstOrderFilter   
{
public:
	FirstOrderFilter(double k) : m_Coef(k) { m_LastValue = 0; }  //����ȡ�����˲�ʱ��Ͳ�������
	double Calc(double curValue)
	{
		double result = (1.0 - m_Coef) * m_LastValue + m_Coef * curValue;
		m_LastValue = result;
		return result;
	}
private:
	double m_Coef;
	double m_LastValue;
};

// λ��ʽPID
class PIDController
{
public:
	PIDController(double kp=0, double ki=0, double kd=0) : m_Kp(kp), m_Ki(ki), m_Kd(kd)
	{
		m_Integral = m_LastError = 0.0;
		m_ThresError1 = m_ThresError2 = DOUBLE_MAX;
		m_ThresIntegral = m_ThresOutput = DOUBLE_MAX;
		m_DeadValue = 0.0;
	}
	//����PID������
	void SetPIDParams(double kp, double ki, double kd)
	{
		m_Kp = kp;
		m_Ki = ki;
		m_Kd = kd;
	}

	//���ÿ�������ֵ
	/*void SetExpectation(double expectation)
	{
		m_Expectation = expectation;
	}*/

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

	void Reset()
	{
		m_Integral = 0;
		m_LastError = 0;
		m_LastTimestamp = std::chrono::high_resolution_clock::time_point();
	}

	double Calc(double expectation, double feedback, std::chrono::high_resolution_clock::time_point curTimestamp, bool flagRadLimit=false)
	{
		double interval = m_LastTimestamp.time_since_epoch().count() == 0 ?
			1 : std::chrono::duration<double, std::milli>(curTimestamp - m_LastTimestamp).count();   // ms
		m_LastTimestamp = curTimestamp;
		double ki = m_Ki * interval;
		double kd = m_Kd / interval;

		//double error = m_Expectation - feedback;
		double error = expectation - feedback;
		if (flagRadLimit)
			error = ClampLoop(error, -PI, PI);  //�ǶȻ����Ƕ���Χ����
		double output = 0.0;
		//���ַ��뷨��������� ��m_ThresError1 ��Χ�����������
		if (fabs(error) >= m_ThresError1)
			m_Integral = 0.0;
		//�������� ��m_ThresError2 ��Χ�ڣ��������֣�����������
		if (fabs(error) < m_ThresError2)
			m_Integral += error;
		//���������޷�����
		m_Integral = Clamp( m_Integral, -m_ThresIntegral, m_ThresIntegral);
		
		output = m_Kp * error + ki * m_Integral + kd * (error - m_LastError);
		if (output > 0)
			output += m_DeadValue;
		else if (output < 0)
			output += -m_DeadValue;
		//��pid������޷�����
		return Clamp(output, -m_ThresOutput, m_ThresOutput);
	}

private:
	double m_Kp, m_Ki, m_Kd;
	std::chrono::high_resolution_clock::time_point m_LastTimestamp; //����PID����һ�����ĵ�ʱ���
	double m_LastError, /*m_Expectation,*/ m_Integral;
	double m_ThresError1, m_ThresError2;  //ThresError1 > ThresError2
	double m_ThresIntegral, m_ThresOutput;
	double m_DeadValue; //������PID���С�ڴ�ֵ��ִ�л���û��Ӧ
};
