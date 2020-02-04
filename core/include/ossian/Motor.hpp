#ifndef OSSIAN_CORE_MOTOR
#define OSSIAN_CORE_MOTOR

#include "Factory.hpp"
#include "IOListener.hpp"

namespace ossian
{

class Motor
{
public:

};

class MotorManager
{
	IOListener* m_IOListener;
public:
	OSSIAN_SERVICE_SETUP(MotorManager(IOListener* ioListener))
		:m_IOListener(ioListener)
	{
	}

	auto AddMotor()
	{
		
	}
};

} // ossian

#endif //OSSIAN_CORE_MOTOR