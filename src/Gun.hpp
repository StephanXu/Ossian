#ifndef OSSIAN_GUN_HPP
#define OSSIAN_GUN_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gimbal.hpp"

#include <chrono>
#include <array>

class Gun
{
public:

	//Ħ����ת������rpm [TODO]ʵ��ó���ͬ�ȼ��µ�������ٶ���������Ӧ��Ħ����ת������
	static double Fric_Speed_12;
	static double Fric_Speed_15;
	static double Fric_Speed_18; 
	static double Fric_Speed_30; 

	//ң��������
	static constexpr size_t SHOOT_MODE_CHANNEL = 1;

	static constexpr uint8_t RC_SW_UP = 1;
	static constexpr uint8_t RC_SW_MID = 3;
	static constexpr uint8_t RC_SW_DOWN = 2;

	//ǹ����������
	static constexpr int HEAT_PER_BULLET = 10;

	//������
	static constexpr int BURST_BULLET_NUM = 3; //������ӵ���������
	static constexpr int FEED_JAM_RPM = 5;     //�жϿ����Ĳ�����ת����ֵ

	static constexpr int FEED_NORMAL_RPM = 10; //�Զ��������Ż�תʱ�������ֹ�����ת��
	static constexpr int FEED_SEMI_RPM = 10;   //����ʱ�������ֹ�����ת��
	static constexpr int FEED_BURST_RPM = 30;  //����ʱ�������ֹ�����ת��
	static constexpr int FEED_AUTO_RPM = 20;   //����ʱ�������ֹ�����ת��   ����>����>����
	


	enum MotorPosition
	{
		FricBelow, FricUpper, Feed
	};

	enum FricMode
	{
		Disable, Enable
	};

	enum FeedMode
	{
		Stop, Reload, Reverse, Semi, Burst, Auto
	};

	OSSIAN_SERVICE_SETUP(Gun(ossian::MotorManager* motorManager, IRemote* remote, Gimbal* gimbal))
		: m_MotorManager(motorManager), m_RC(remote), m_Gimbal(gimbal)
	{
		m_FlagInitFric = m_FlagInitFeed = true;

		PIDController pidFricSpeed(15000, 10, 0);
		pidFricSpeed.SetThresOutput(16384);
		pidFricSpeed.SetThresIntegral(2000);
		m_PIDFricSpeed.fill(pidFricSpeed);

		m_PIDFeedSpeed.SetPIDParams(800, 0.5, 0);
		m_PIDFeedSpeed.SetThresOutput(10000);
		m_PIDFeedSpeed.SetThresIntegral(9000);

	}

	void InitFric()
	{
		m_FricMode = FricMode::Disable;
		m_FricMotorMsgCheck.fill(false);

		std::for_each(m_PIDFricSpeed.begin(), m_PIDFricSpeed.end(), [](PIDController& x) {x.Reset(); });

		m_FlagInitFric = false;
	}

	void InitFeed()
	{
		m_FeedMode = FeedMode::Stop;

		m_PIDFeedSpeed.Reset();

		m_FlagInitFeed = false;
	}

	auto AddMotor(MotorPosition position,
		const std::string location,
		const unsigned int motorId,
		const unsigned int writerCanId)->void
	{
		if (position == FricBelow || position == FricUpper)
			m_Motors[position] =
			m_MotorManager->AddMotor<ossian::DJIMotor>(
				location,
				m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
				[this, position](const std::shared_ptr<ossian::DJIMotor>& motor)
				{
					MotorFricReceiveProc(motor, position);
				},
				motorId);
		else if (position == Feed)
			m_Motors[position] =
				m_MotorManager->AddMotor<ossian::DJIMotor>(
					location,
					m_MotorManager->GetOrAddWriter<ossian::DJIMotorWriter>(location, writerCanId),
					[this, position](const std::shared_ptr<ossian::DJIMotor>& motor)
					{
						MotorFeedReceiveProc(motor, position);
					},
					motorId);
	}

	void UpdateGunSensorFeedback()
	{
		m_GunSensorValues.rc = m_RC->Status();
		m_GunSensorValues.gimbalInputSrc = m_Gimbal->GimbalCtrlSrc();  //[TODO] ���Ӷ�����ģʽ����Ĵ���

		//[TODO] �ȶ�ʱ��������������ݣ�0x0207���и��£����ۼ��ѷ�����ӵ���
		//[TODO] �����ǰ��ȡ������������ʷ���������ѷ�����ӵ�������
	}
	bool MicroSwitchStatus() { return false; } //��ȡǹ��΢������ or ���Թܵ�״̬

	void FricModeSet();

	void FricExpSpeedSet();

	void FricCtrl();

	void FeedModeSet();

	//���͵�����2006
	void FeedRotateCtrl(bool stop=false, int rpmSet=0, bool reverse=false);

	void AutoReloadCtrl();

	void SingleShotCtrl(int rpmSet);

	void FeedCtrl();

	auto MotorFricReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor, MotorPosition position)->void
	{
		m_FricMotorMsgCheck[position] = true;
		if (!(m_FricMotorMsgCheck[FricBelow] && m_FricMotorMsgCheck[FricUpper]))
			return;

		UpdateGunSensorFeedback();
		if (m_FlagInitFric)
			InitFric();

		FricModeSet();
		FricExpSpeedSet();
		FricCtrl();
		m_FricMotorMsgCheck.fill(false);
	}

	auto MotorFeedReceiveProc(const std::shared_ptr<ossian::DJIMotor>& motor, MotorPosition position)->void
	{
		UpdateGunSensorFeedback();
		if (m_FlagInitFeed)
			InitFeed();

		FeedModeSet();
		FeedCtrl();
	}


private:
	ossian::MotorManager* m_MotorManager;
	std::array<std::shared_ptr<ossian::DJIMotor>, 3> m_Motors;
	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	IRemote* m_RC;  //ң����
	Gimbal* m_Gimbal;

	struct GunSensorFeedback
	{
		RemoteStatus rc;	 //ң��������
		Gimbal::GimbalInputSrc gimbalInputSrc;
		double refereeCurHeat, refereeHeatLimit;
		int shooter_heat0_speed_limit;
		std::chrono::high_resolution_clock::time_point refereeTimeStamp;
	} m_GunSensorValues;
	
	bool m_FlagInitFric, m_FlagInitFeed;
	std::array<bool, 2> m_FricMotorMsgCheck;
	FricMode m_FricMode;
	FeedMode m_FeedMode;

	int m_CurBulletShotNum; //���������������Ĺ����У��ۻ�������ӵ���
	double m_FricSpeedSet;
	std::array<PIDController, 2> m_PIDFricSpeed;
	PIDController m_PIDFeedSpeed;
};

#endif // OSSIAN_GUN_HPP