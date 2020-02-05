#ifndef OSSIAN_CHASSIS_HPP
#define OSSIAN_CHASSIS_HPP

#include <ossian/Motor.hpp>

#include <chrono>

class Chassis
{
	ossian::MotorManager* m_MotorManager;

	std::array<std::shared_ptr<ossian::IMotor>, 4> m_Motors;
	
	std::chrono::high_resolution_clock::time_point m_LastRefresh;
	
public:
	OSSIAN_SERVICE_SETUP(Chassis(ossian::MotorManager* motorManager))
		: m_MotorManager(motorManager)
	{
	}

	enum MotorPosition
	{
		FrontLeft = 0,
		FrontRight,
		BehindLeft,
		BehindRight
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

	auto MotorReceiveProc(std::shared_ptr<ossian::DJIMotor> motor)->void
	{

	}
};

#endif // OSSIAN_CHASSIS_HPP