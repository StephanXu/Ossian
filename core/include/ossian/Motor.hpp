#ifndef OSSIAN_CORE_MOTOR
#define OSSIAN_CORE_MOTOR

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>

#include "Factory.hpp"
#include "io/CAN.hpp"

namespace ossian
{

class IMotorWriter;

class IMotor
{
public:
	virtual ~IMotor() = default;
	virtual auto Parse(const uint8_t* buffer, const size_t bufferSize)->void = 0;
	virtual auto MotorId() const noexcept -> unsigned int = 0;
	virtual auto CANId() const noexcept -> unsigned int = 0;
	virtual auto TimeStamp() const noexcept -> std::chrono::high_resolution_clock::time_point = 0;
	virtual auto Voltage() const noexcept -> uint16_t = 0;
	virtual auto SetVoltage(const uint16_t voltage) noexcept -> void = 0;
	virtual auto Writer() const noexcept -> const IMotorWriter* = 0;
protected:
	virtual auto SetMotorId(const unsigned int id) noexcept -> void = 0;
	virtual auto SetWriter(const IMotorWriter* writer) -> void = 0;
};

class IMotorWriter
{
public:
	virtual ~IMotorWriter() = default;
	virtual auto AddDevice(const std::shared_ptr<IMotor>& motor) -> void = 0;
	virtual auto PackAndSend() const -> void = 0;
	virtual auto Location() const->std::string = 0;
	virtual auto CANId() const noexcept ->unsigned int = 0;
protected:
	virtual auto SetDevice(const std::shared_ptr<CANDevice> device) noexcept ->void = 0;
};

template<class MotorType>
using MotorCallback = std::function<void(const std::shared_ptr<MotorType> & motor)>;

class MotorManager
{
	CANManager* m_CANManager;
public:
	OSSIAN_SERVICE_SETUP(MotorManager(CANManager* canManager))
		:m_CANManager(canManager)
	{
	}

	~MotorManager() = default;

	template<class MotorType, class ...Args>
	auto AddMotor(const std::string location,
				  const std::shared_ptr<IMotorWriter>& writer,
				  MotorCallback<MotorType> callback,
				  Args... args)
	{
		static_assert(std::is_base_of<IMotor, MotorType>::value, "MotorType has to implement IMotor");
		auto parseProc = [this, callback](const std::shared_ptr<CANDevice>& device,
										  const size_t length,
										  const uint8_t* data)
		{
			spdlog::info("MotorManager Received: {}", length);
			auto motor = std::static_pointer_cast<MotorType>(m_Motors[device]);
			motor->Parse(data, length);
			callback(motor);
		};
		auto motor = std::make_shared<MotorType>(std::forward<Args>(args)...);
		auto dev = m_CANManager->AddDevice(location, motor->CANId())->SetCallback(parseProc);
		writer->AddDevice(motor);
		motor->SetWriter(writer.get());
		m_Motors.insert(std::make_pair(dev, motor));
		
		spdlog::trace("Add new motor: location: {}, motorId: {}, CANId: {:#x},",
					  location,
					  motor->MotorId(),
					  motor->CANId());
		
		return motor;
	}

	template<class WriterType, class...Args>
	auto GetOrAddWriter(const std::string location,
						Args...args)->std::shared_ptr<IMotorWriter>
	{
		static_assert(std::is_base_of<IMotorWriter, WriterType>::value, "WriterType has to implement IMotorWriter");
		auto writer = std::make_shared<WriterType>(std::forward<Args>(args)...);
		auto dev = m_CANManager->AddDevice(location, writer->CANId());
		auto it = m_Writers.find(dev);
		if (it != m_Writers.end())
		{
			return it->second;
		}
		writer->SetDevice(dev);
		m_Writers.insert(std::make_pair(dev, writer));

		spdlog::trace("Add new motor writer: location: {}, CANId: {:#x}", location, writer->CANId());

		return std::dynamic_pointer_cast<IMotorWriter>(writer);
	}

private:
	using Container = std::unordered_map<std::shared_ptr<BaseDevice>, std::shared_ptr<IMotor>>;
	using WriterContainer = std::unordered_map<std::shared_ptr<BaseDevice>, std::shared_ptr<IMotorWriter>>;
	
	Container m_Motors;
	WriterContainer m_Writers;
};

class DJIMotorWriter : public IMotorWriter
{
	friend class MotorManager;

#pragma pack(push,1)
	struct PackModel
	{
		uint16_t m_Voltage[4];
	};
#pragma pack(pop)

public:
	DJIMotorWriter(const unsigned int canId)
		:m_Motors(m_MotorsNum)
		, m_CANId(canId)
	{
	}

	auto AddDevice(const std::shared_ptr<IMotor>& motor) -> void override
	{
		const size_t index{ motor->MotorId() - (0x1ff == m_CANId ? 1 : 5) };
		if (index > m_Motors.size()) { throw std::runtime_error("Invalid motor id"); }
		m_Motors[index] = motor;
		spdlog::trace("MotorWriter add device: writerCANId: {:#x}, motorId: {}", m_CANId, motor->MotorId());
	}

	auto Pack(PackModel& outModel) const noexcept -> void
	{
		for (size_t i{}; i < m_MotorsNum; ++i)
		{
			outModel.m_Voltage[i] = 0;
			outModel.m_Voltage[i] |= (m_Motors[i]->Voltage() >> 8) & 0x00ff;
			outModel.m_Voltage[i] |= (m_Motors[i]->Voltage() << 8) & 0xff00;
		}
	}

	auto Send(const PackModel& refModel) const -> void
	{
		if (m_Device)
		{
			m_Device->WriteRaw(sizeof(PackModel), reinterpret_cast<const uint8_t*>(&refModel));
		}
	}

	auto PackAndSend() const -> void override
	{
		PackModel buffer{};
		Pack(buffer);
		Send(buffer);
	}

	auto CANId() const noexcept -> unsigned int override
	{
		return m_CANId;
	}

	auto Location() const -> std::string
	{
		if (!m_Device) { throw std::runtime_error("Empty device"); }
		return m_Device->Bus()->Location();
	}

private:

	auto SetDevice(const std::shared_ptr<CANDevice> device) noexcept -> void override
	{
		m_Device = device;
	}

	const unsigned int m_MotorsNum = 4;
	std::vector<std::shared_ptr<IMotor>> m_Motors;
	unsigned int m_CANId;
	std::shared_ptr<CANDevice> m_Device;
};

class DJIMotor : public IMotor
{
	friend class MotorManager;
	using DefaultWriter = DJIMotorWriter;
	const unsigned int CANIdBase = 0x200;
public:
	DJIMotor(unsigned int motorId)
		:m_MotorId(motorId)
	{
	}

#pragma pack(push,1)
	struct ReceiveModel
	{
		uint16_t m_Encoding;
		int16_t m_RPM;
		int16_t m_Current;
		uint8_t m_Temperature;
		uint8_t m_Reserve;
	};
#pragma pack(pop)

	static auto Parse(ReceiveModel& outModel,
					  const uint8_t* buffer,
					  const size_t bufferSize)
	{
		if (buffer)
		{
			std::copy(buffer,
					  buffer + bufferSize,
					  reinterpret_cast<uint8_t*>(&outModel));
		}
	}

	auto Parse(const uint8_t* buffer, const size_t bufferSize)->void override
	{
		m_TimeStamp = std::chrono::high_resolution_clock::now();
		Parse(m_Status, buffer, bufferSize);
	}

	auto Status() noexcept -> ReceiveModel&
	{
		return m_Status;
	}

	auto MotorId() const noexcept -> uint32_t override
	{
		return m_MotorId;
	}

	auto CANId() const noexcept -> unsigned int override
	{
		return m_MotorId + CANIdBase;
	}

	auto TimeStamp() const noexcept -> std::chrono::high_resolution_clock::time_point override
	{
		return m_TimeStamp;
	}

	auto Voltage() const noexcept -> uint16_t override
	{
		return m_Voltage;
	}

	auto SetVoltage(const uint16_t voltage) noexcept -> void override
	{
		m_Voltage = voltage;
	}

	auto Writer() const noexcept->const IMotorWriter* override
	{
		return m_Writer;
	}

private:
	ReceiveModel m_Status = {};
	std::chrono::high_resolution_clock::time_point m_TimeStamp;
	unsigned int m_MotorId;
	uint16_t m_Voltage = 0;
	const IMotorWriter* m_Writer;

	auto SetMotorId(const unsigned int id) noexcept -> void override
	{
		m_MotorId = id;
	}

	auto SetWriter(const IMotorWriter* writer)->void override
	{
		m_Writer = writer;
	}
};

} // ossian

#endif //OSSIAN_CORE_MOTOR