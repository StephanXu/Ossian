#ifndef OSSIAN_CORE_MOTOR
#define OSSIAN_CORE_MOTOR

#include <chrono>

#include "Factory.hpp"
#include "io/CAN.hpp"

namespace ossian
{

class IMotor
{
public:
	virtual auto Parse(std::shared_ptr<uint8_t[]> buffer, size_t bufferSize)->void = 0;
};

template<class MotorType>
using MotorCallback = std::function<void(std::shared_ptr<MotorType> motor)>;

class MotorManager
{
	CANManager* m_CANManager;
public:
	OSSIAN_SERVICE_SETUP(MotorManager(CANManager* canManager))
		:m_CANManager(canManager)
	{
	}

	template<class MotorType>
	auto AddMotor(const std::string location,
				  const unsigned int id,
				  MotorCallback<MotorType> callback)
	{
		static_assert(std::is_base_of<IMotor, MotorType>::value, "MotorType has to implement IMotor");
		auto parseProc = [this, callback](std::shared_ptr<BaseDevice> device,
										  size_t length,
										  std::shared_ptr<uint8_t[]>data)
		{
			auto motor = std::static_pointer_cast<MotorType>(m_Motors[device]);
			motor->Parse(data, length);
			callback(motor);
		};
		auto dev = m_CANManager->AddDevice(location, id, parseProc);
		m_Motors.insert(std::make_pair(dev, std::make_shared<MotorType>()));
	}
	
private:
	using Container = std::unordered_map<std::shared_ptr<BaseDevice>, std::shared_ptr<IMotor>>;
	Container m_Motors;
};

class DJIMotor : public IMotor
{
public:
#pragma pack(push,1)
	struct ReceiveModel
	{
		uint16_t m_Angle;
		int16_t m_Speed;
		int16_t m_Current;
		uint8_t m_Temperature;
		uint8_t m_Reserve;
	};
#pragma pack(pop)

	static auto Parse(ReceiveModel& outModel,
					  std::shared_ptr<uint8_t[]> buffer,
					  const size_t bufferSize)
	{
		if (buffer)
		{
			std::copy(buffer.get(),
					  buffer.get() + bufferSize,
					  reinterpret_cast<uint8_t*>(&outModel));
		}
	}

	auto Parse(std::shared_ptr<uint8_t[]> buffer, const size_t bufferSize)->void override
	{
		m_TimeStamp = std::chrono::high_resolution_clock::now();
		Parse(m_Status, buffer, bufferSize);
	}

	auto Status()->ReceiveModel& { return m_Status; }

private:
	ReceiveModel m_Status = {};
	std::chrono::high_resolution_clock::time_point m_TimeStamp;
};

} // ossian

#endif //OSSIAN_CORE_MOTOR