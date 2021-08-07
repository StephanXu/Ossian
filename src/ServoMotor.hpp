#ifndef OSSIAN_SERVO_MOTOR
#define OSSIAN_SERVO_MOTOR

#include <ossian/Factory.hpp>

class ServoMotor
{
	static constexpr char PIPE_NAME[] = "servo_motor_fifo";
public:
	OSSIAN_SERVICE_SETUP(ServoMotor());

	auto SetPWMValue(uint8_t channel, double value) -> void;

	auto IsValid() const -> bool;

private:
	int m_FD     = 0;
	bool m_Valid = false;

	auto OpenFifo() -> void;
};

#endif // OSSIAN_SERVO_MOTOR
