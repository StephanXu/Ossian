#ifndef OSSIAN_GUN_HPP
#define OSSIAN_GUN_HPP

#include <ossian/Motor.hpp>
#include "CtrlAlgorithms.hpp"
#include "InputAdapter.hpp"
#include "Remote.hpp"
#include "Gimbal.hpp"

#include <chrono>
#include <array>
#include <thread>

class Gun
{
public:

	//Ħ����ת������rpm [TODO]ʵ��ó���ͬ�ȼ��µ�������ٶ���������Ӧ��Ħ����ת������
	static int16_t kFricSpeed12;
	static int16_t kFricSpeed15;
	static int16_t kFricSpeed18;
	static int16_t kFricSpeed30;

	//ң��������
	static constexpr int16_t kGunRCDeadband = 50; //��������
	static constexpr size_t kShootModeChannel = 4;

	static constexpr uint8_t kRCSwUp = 1;
	static constexpr uint8_t kRCSwMid = 3;
	static constexpr uint8_t kRCSwDown = 2;

	//ǹ����������
	static constexpr int kHeatPerBullet = 10;

	//������
	static constexpr int kBurstBulletNum = 3; //������ӵ���������
	static constexpr int16_t kFeedJamRPM = 5;     //�жϿ����Ĳ�����ת����ֵ

	static int16_t kFeedNormalRPM; //�Զ��������Ż�תʱ�������ֹ�����ת��
	static int16_t kFeedSemiRPM;   //����ʱ�������ֹ�����ת��
	static int16_t kFeedBurstRPM;  //����ʱ�������ֹ�����ת��
	static int16_t kFeedAutoRPM;   //����ʱ�������ֹ�����ת��   ����>����>����
	
	//pid����
	static std::array<double, 5> PIDFricSpeedParams;
	static std::array<double, 5> PIDFeedSpeedParams;

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

	OSSIAN_SERVICE_SETUP(Gun(ossian::MotorManager* motorManager, IRemote* remote, Gimbal* gimbal, Utils::ConfigLoader* config))
		: m_MotorManager(motorManager), m_RC(remote), m_Gimbal(gimbal), m_Config(config)
	{
		using OssianConfig::Configuration;
		PIDFricSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->kp();
		PIDFricSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->ki();
		PIDFricSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->kd();
		PIDFricSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->thout();
		PIDFricSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidfricspeed()->thiout();

		PIDFeedSpeedParams[0] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->kp();
		PIDFeedSpeedParams[1] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->ki();
		PIDFeedSpeedParams[2] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->kd();
		PIDFeedSpeedParams[3] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->thout();
		PIDFeedSpeedParams[4] = m_Config->Instance<Configuration>()->mutable_pidfeedspeed()->thiout();

		kFricSpeed12 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed12();
		kFricSpeed15 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed15();
		kFricSpeed18 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed18();
		kFricSpeed30 = m_Config->Instance<Configuration>()->mutable_gun()->kfricspeed30();

		kFeedNormalRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeednormalrpm();
		kFeedSemiRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeedsemirpm();
		kFeedBurstRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeedburstrpm();
		kFeedAutoRPM = m_Config->Instance<Configuration>()->mutable_gun()->kfeedautorpm();


		m_FlagInitFric = m_FlagInitFeed = true;

		PIDController pidFricSpeed;
		pidFricSpeed.SetParams(PIDFricSpeedParams);
		m_PIDFricSpeed.fill(pidFricSpeed);

		m_PIDFeedSpeed.SetParams(PIDFeedSpeedParams);

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
	Utils::ConfigLoader* m_Config;
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
	int16_t m_FricSpeedSet;
	std::array<PIDController, 2> m_PIDFricSpeed;
	PIDController m_PIDFeedSpeed;
};

#endif // OSSIAN_GUN_HPP