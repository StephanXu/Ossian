#ifndef CTRLALGORITHMS_HPP
#define CTRLALGORITHMS_HPP

#include <cmath>
#include <limits>
#include <chrono>
#include <array>
#include <Eigen/Dense>

using hrClock = std::chrono::high_resolution_clock;
// 限幅函数
template<typename T>
inline T Clamp(T value, const T& lowerBnd, const T& upperBnd)
{
	if (value > upperBnd)
		return upperBnd;
	else if (value < lowerBnd)
		return lowerBnd;
	else
		return value;
}

//死区限制
template<typename T>
inline T DeadbandLimit(T value, const T& deadband)
{
	if (value > deadband || value < -deadband)
		return value;
	else
		return 0;
}

//循环限幅
template<typename T>
inline T ClampLoop(T value, const T& lowerBnd, const T& upperBnd)
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


//计算当前编码值与中值之间的相对角度rad
inline double RelativeEcdToRad(const uint16_t& ecd, const uint16_t& ecdMid)
{
	static constexpr uint16_t kHalfEcdRange = 4096;
	static constexpr uint16_t kEcdRange = 8191;
	static constexpr double kMotorEcdToRadCoef = 2 * M_PI / 8192;

	int relativeEcd = static_cast<int>(ecd) - static_cast<int>(ecdMid);
	if (relativeEcd > kHalfEcdRange)
		relativeEcd -= kEcdRange;
	else if (relativeEcd < -kHalfEcdRange)
		relativeEcd += kEcdRange;
	return relativeEcd * kMotorEcdToRadCoef;
}
// out = k / (k + T) * out + T / (k + T) * in
// 一阶低通滤波器
class FirstOrderFilter   
{
public:
	FirstOrderFilter(double k=1,double t=1) : m_Coef(k),m_FramePeriod(t) { m_LastResult = 0; }  //参数取决于滤波时间和采样周期
	void SetState(double k, double t) { m_Coef = k; m_FramePeriod = t; }
	double Calc(double curValue)
	{
		double result = m_Coef / (m_Coef + m_FramePeriod) * m_LastResult + m_FramePeriod / (m_Coef + m_FramePeriod) * curValue;
		m_LastResult = result;
		return result;
	}

	void Reset() { m_LastResult = 0; }

private:
	double m_Coef;
	double m_LastResult;
	double m_FramePeriod;
};

// 位置式PID
class PIDController
{
public:
	static constexpr double DOUBLE_MAX = std::numeric_limits<double>::max() - 5;
	PIDController(double kp=0, double ki=0, double kd=0) : m_Kp(kp), m_Ki(ki), m_Kd(kd)
	{
		m_Integral = m_LastError = 0.0;
		m_ThresError1 = m_ThresError2 = DOUBLE_MAX;
		m_ThresIntegral = m_ThresOutput = DOUBLE_MAX;
		m_DeadValue = 0.0;
		m_FlagAngleLoop = false;
		m_CtrlInterval = 1;
	}

	//设置PID五参数
	void SetParams(const std::array<double, 5>& params)
	{
		m_Kp = params[0];
		m_Ki = params[1];
		m_Kd = params[2];
		m_ThresOutput = params[3];
		//若设置负数，则代表不设积分限制
		if (params[4] >= 0)
			m_ThresIntegral = params[4];
	}
	void SetkCtrlFreq(const double& freq)
	{
		m_CtrlInterval = 1000.0 / freq;  //ms
	}
	void SetFlagAngleLoop()
	{
		m_FlagAngleLoop = true;
	}
	void SetDeadBand(const double& db)
	{
		m_DeadValue = db;
	}
	void SetThresError(const double& th1, const double& th2)
	{
		m_ThresError1 = th1;
		m_ThresError2 = th2;
	}

	void Reset()
	{
		m_Integral = 0;
		m_LastError = 0;
		//m_LastTimestamp = hrClock::time_point();
	}

	double Calc(double expectation, double feedback)
	{
		/*double interval = (m_LastTimestamp.time_since_epoch().count() == 0 ?
			1 : std::chrono::duration<double, std::milli>(curTimestamp - m_LastTimestamp).count());   // ms
		m_LastTimestamp = curTimestamp; */
		double ki = m_Ki * m_CtrlInterval;
		double kd = m_Kd / m_CtrlInterval;

		//double error = m_Expectation - feedback;
		double error = expectation - feedback;
		if (m_FlagAngleLoop)
		{
			error = ClampLoop(error, -M_PI, M_PI);  //角度环，角度误差范围限制
			error = DeadbandLimit(error, 0.01);
		}
			
		double output = 0.0;
		//积分分离法：如果误差超过 ±m_ThresError1 范围，则积分清零
		if (fabs(error) >= m_ThresError1)
			m_Integral = 0.0;
		//如果误差在 ±m_ThresError2 范围内，则做积分，否则不做积分
		if (fabs(error) < m_ThresError2)
			m_Integral += error;
		//将积分做限幅处理
		m_Integral = Clamp( m_Integral, -m_ThresIntegral, m_ThresIntegral);
		//SPDLOG_INFO("@PIDInterval=[$interval={}]", interval);
		//SPDLOG_INFO("@Integral=[$i={}]", m_Integral);
		output = m_Kp * error + ki * m_Integral + kd * (error - m_LastError);

		if (output > 0)
			output += m_DeadValue;
		else if (output < 0)
			output += -m_DeadValue;

		m_LastError = error;
		//将pid输出做限幅处理
		return Clamp(output, -m_ThresOutput, m_ThresOutput);
	}

private:
	double m_Kp, m_Ki, m_Kd;
	//hrClock::time_point m_LastTimestamp; //输入PID的上一条报文的时间戳
	double m_LastError, /*m_Expectation,*/ m_Integral;
	double m_ThresError1, m_ThresError2;  //ThresError1 > ThresError2
	double m_ThresIntegral, m_ThresOutput;
	double m_DeadValue; //死区：PID输出小于此值，执行机构没反应
	bool m_FlagAngleLoop;
	double m_CtrlInterval;  //ms
};

class KalmanFilter
{
public:
	KalmanFilter(int stateNum, int measureNum, int controlNum = 0) : n(stateNum), m(measureNum), u(controlNum) {}

	//n=4 m=2
	void SetMatsForAutoAim(double dt)
	{
		A = Eigen::MatrixXd(4, 4);
		A << 1, 0, dt,  0,
			 0, 1,  0, dt,
			 0, 0,  1,  0,
			 0, 0,  0,  1;

		H = H.Identity(m,n);

		Q.resize(n, n);
		Q.fill(1e-2); //bigger ---- slower regression

		R.resize(m, m);
		R.fill(1e-3); //smaller --- quicker regression

		P0.resize(n, n);
		P0.fill(1);

		X0.resize(n);
		X0.setRandom();
	}
	
	void SetFixedMat(Eigen::MatrixXd _A, Eigen::MatrixXd _H, Eigen::MatrixXd _Q, Eigen::MatrixXd _R, Eigen::MatrixXd _B = Eigen::MatrixXd())
	{
		A = _A;
		H = _H;
		Q = _Q;
		R = _R;
		B = _B;
		I = I.Identity(n, n);
	}

	void SetInitialState(Eigen::VectorXd _X0, Eigen::MatrixXd _P0)
	{
		X0 = _X0;
		P0 = _P0;
	}

	Eigen::VectorXd Predict()
	{
		X = (A * X0);
		P = (A * P0 * A.transpose()) + Q;
		return X;
	}

	//U：控制向量
	Eigen::VectorXd Predict(Eigen::VectorXd U)
	{
		X = (A * X0) + (B * U);
		P = (A * P0 * A.transpose()) + Q;
		return X;
	}

	//Z：观测向量
	void Correct(Eigen::VectorXd Z)
	{
		K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
		X = X + K * (Z - H * X);
		P = (I - K * H) * P;

		X0 = X;
		P0 = P;
	}

private:
	//问题规模
	int n; //状态数
	int m; //观测量数
	int u; //控制向量维度

	//固定矩阵
	Eigen::MatrixXd A; //状态转移矩阵
	Eigen::MatrixXd B; //控制矩阵
	Eigen::MatrixXd H; //观测矩阵
	Eigen::MatrixXd Q; //过程噪声协方差矩阵
	Eigen::MatrixXd R; //测量噪声协方差矩阵
	Eigen::MatrixXd I; //单位矩阵

	//变量矩阵
	Eigen::VectorXd X; //当前时刻状态向量
	Eigen::MatrixXd P; //状态协方差矩阵
	Eigen::MatrixXd K; //卡尔曼增益矩阵

	//初始状态
	Eigen::VectorXd X0; //初始状态向量
	Eigen::MatrixXd P0; //初始状态协方差矩阵

/*
*	A: n x n
*	B: n x u
*	H: m x n
*	Q: n x n
*	R: m x m
*	I: n x n
*	X: n x 1
*	U: u x 1
*	Z: m x 1
*	P: n x n
*	K: n x m
*
*/
};


#endif //CTRLALGORITHMS_HPP