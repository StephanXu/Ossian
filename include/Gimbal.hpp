#ifndef OSSIAN_GIMBAL_HPP
#define OSSIAN_GIMBAL_HPP

#include <ossian/Motor.hpp>

#include <chrono>

class Gimbal
{
	ossian::MotorManager* m_MotorManager;

	std::array<std::shared_ptr<ossian::IMotor>, 2> m_Motors;

	std::chrono::high_resolution_clock::time_point m_LastRefresh;
public:
	OSSIAN_SERVICE_SETUP(Gimbal(ossian::MotorManager* motorManager))
		: m_MotorManager(motorManager)
	{
	}

	enum MotorPosition
	{
		Pitch = 0,
		Yaw
	};

	auto AddMotor(MotorPosition position,
				  const std::string location,
				  const unsigned int id)
	{
		m_MotorManager->AddMotor<ossian::DJIMotor>(
			location,
			id,
			[this](std::shared_ptr<ossian::DJIMotor> motor)
			{
				MotorReceiveProc(motor);
			});
	}

	float m_CurrentPitch;
	float m_CurrentYaw;
	
	auto MoveYaw(const float delta)->void
	{
		m_CurrentYaw += delta;
	}
	
	auto MotorReceiveProc(std::shared_ptr<ossian::DJIMotor> motor)->void
	{

	}
};

#endif // OSSIAN_GIMBAL_HPP