#ifndef OSSIAN_CORE_MOTOR
#define OSSIAN_CORE_MOTOR

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>

#include "../Factory.hpp"
#include "../io/CAN.hpp"

namespace ossian
{
class IMotorWriter;

class IMotor
{
public:
	virtual ~IMotor() = default;
	virtual auto Parse(const uint8_t* buffer, const size_t bufferSize) -> void = 0;
	virtual auto MotorId() const noexcept -> unsigned int = 0;
	virtual auto CANId() const noexcept -> unsigned int = 0;
	virtual auto TimeStamp() const noexcept -> std::chrono::high_resolution_clock::time_point = 0;
	virtual auto Voltage() const noexcept -> int16_t = 0;
	virtual auto SetVoltage(const int16_t voltage) noexcept -> void = 0;
	virtual auto Writer() const noexcept -> IMotorWriter* = 0;
	virtual auto Lock() -> void = 0;
	virtual auto UnLock() -> void = 0;
	virtual auto TryLock() noexcept -> bool = 0;
protected:
	virtual auto SetMotorId(const unsigned int id) noexcept -> void = 0;
	virtual auto SetWriter(IMotorWriter* writer) -> void = 0;
};

class IMotorWriter
{
public:
	virtual ~IMotorWriter() = default;
	virtual auto AddDevice(const std::shared_ptr<IMotor>& motor) -> void = 0;
	virtual auto PackAndSend() -> void = 0;
	virtual auto Location() const -> std::string = 0;
	virtual auto CANId() const noexcept -> unsigned int = 0;
protected:
	virtual auto SetDevice(const std::shared_ptr<CANDevice> device) noexcept -> void = 0;
};

template <class MotorType>
using MotorCallback = std::function<void(const std::shared_ptr<MotorType>& motor)>;

class MotorManager
{
	CANManager* m_CANManager;
public:
	OSSIAN_SERVICE_SETUP(MotorManager(CANManager* canManager))
		: m_CANManager(canManager)
	{
	}

	~MotorManager() = default;

	template <class MotorType, class ...Args>
	auto AddMotor(const std::string location,
	              const std::shared_ptr<IMotorWriter>& writer,
	              MotorCallback<MotorType> callback,
	              Args ... args)
	{
		static_assert(std::is_base_of<IMotor, MotorType>::value, "MotorType has to implement IMotor");
		auto parseProc = [this, callback](const std::shared_ptr<CANDevice>& device,
		                                  const size_t length,
		                                  const uint8_t* data)
		{
			//SPDLOG_TRACE("MotorManager Received: {}", length);
			auto motor = std::static_pointer_cast<MotorType>(m_Motors[device]);
			motor->Parse(data, length);
			callback(motor);
		};
		auto motor = std::make_shared<MotorType>(std::forward<Args>(args)...);
		auto dev   = m_CANManager->AddDevice(location, motor->CANId())->SetCallback(parseProc);
		writer->AddDevice(motor);
		motor->SetWriter(writer.get());
		m_Motors.insert(std::make_pair(dev, motor));

		SPDLOG_TRACE("Add new motor: location: {}, motorId: {}, CANId: {:#x},",
		             location,
		             motor->MotorId(),
		             motor->CANId());

		return motor;
	}

	template <class WriterType, class...Args>
	auto GetOrAddWriter(const std::string location,
	                    Args ...args) -> std::shared_ptr<IMotorWriter>
	{
		static_assert(std::is_base_of<IMotorWriter, WriterType>::value, "WriterType has to implement IMotorWriter");
		auto writer = std::make_shared<WriterType>(std::forward<Args>(args)...);
		auto dev    = m_CANManager->AddDevice(location, writer->CANId());
		auto it     = m_Writers.find(dev);
		if (it != m_Writers.end())
		{
			return it->second;
		}
		writer->SetDevice(dev);
		m_Writers.insert(std::make_pair(dev, writer));

		SPDLOG_TRACE("Add new motor writer: location: {}, CANId: {:#x}", location, writer->CANId());

		return std::dynamic_pointer_cast<IMotorWriter>(writer);
	}

private:
	using Container = std::unordered_map<std::shared_ptr<BaseDevice>, std::shared_ptr<IMotor>>;
	using WriterContainer = std::unordered_map<std::shared_ptr<BaseDevice>, std::shared_ptr<IMotorWriter>>;

	Container m_Motors;
	WriterContainer m_Writers;
};
} // ossian

#endif //OSSIAN_CORE_MOTOR
