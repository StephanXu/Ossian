#ifndef OSSIAN_CORE_MOTORS_DJI_MOTOR
#define OSSIAN_CORE_MOTORS_DJI_MOTOR

#include <mutex>

#include "Motor.hpp"
#include "../Utility.hpp"
#include "../MultiThread.hpp"
#include "../io/CAN.hpp"

namespace ossian
{

template<size_t NumMotors>
struct MultipleMotorsStatus
{
	uint16_t m_Encoding[NumMotors];
	int16_t m_RPM[NumMotors];
};

#pragma pack(push,1)

struct ReceiveModel3508
{
	uint16_t m_Encoding;
	int16_t m_RPM;
	int16_t m_Current;
	uint8_t m_Temperature;
	uint8_t m_Reserve;

	static auto Parse(ReceiveModel3508& outModel,
					  const uint8_t* buffer,
					  const size_t bufferSize)
	{
		if (buffer)
		{
			std::copy(buffer,
					  buffer + bufferSize,
					  reinterpret_cast<uint8_t*>(&outModel));
			outModel.m_Encoding = ConvertEndian(outModel.m_Encoding);
			outModel.m_RPM = ConvertEndian(outModel.m_RPM);
			outModel.m_Current = ConvertEndian(outModel.m_Current);
		}
	}
};

struct ReceiveModel2006
{
	uint16_t m_Encoding;
	int16_t m_RPM;
	int16_t m_Torque;
	uint16_t m_Reserve;

	static auto Parse(ReceiveModel2006& outModel,
					  const uint8_t* buffer,
					  const size_t bufferSize)
	{
		if (buffer)
		{
			std::copy(buffer,
					  buffer + bufferSize,
					  reinterpret_cast<uint8_t*>(&outModel));
			outModel.m_Encoding = ConvertEndian(outModel.m_Encoding);
			outModel.m_RPM = ConvertEndian(outModel.m_RPM);
			outModel.m_Torque = ConvertEndian(outModel.m_Torque);
		}
	}
};

#pragma pack(pop)

template <size_t DefaultWriterCANId,
	typename Mutex = std::mutex>
	class DJIMotorWriter : public IMotorWriter
{
	friend class MotorManager;

#pragma pack(push,1)
	struct PackModel
	{
		int16_t m_Voltage[4];
	};
#pragma pack(pop)

public:
	DJIMotorWriter(const unsigned int canId)
		: m_Motors(m_MotorsNum)
		, m_CANId(canId)
	{
	}

	auto AddDevice(const std::shared_ptr<IMotor>& motor) -> void override
	{
		std::lock_guard<Mutex> guard{ m_Mutex };
		const size_t index{ motor->MotorId() - (DefaultWriterCANId == m_CANId ? 1 : 5) };
		if (index > m_Motors.size()) { throw std::runtime_error("Invalid motor id"); }
		m_Motors[index] = motor;
		SPDLOG_TRACE("MotorWriter add device: writerCANId: {:#x}, motorId: {}", m_CANId, motor->MotorId());
	}

	auto Pack(PackModel& outModel) noexcept -> void
	{
		for (size_t i{}; i < m_MotorsNum; ++i)
		{
			if (m_Motors[i])
			{
				m_Motors[i]->Lock();
				outModel.m_Voltage[i] = 0;
				outModel.m_Voltage[i] |= (m_Motors[i]->Voltage() >> 8) & 0x00ff;
				outModel.m_Voltage[i] |= (m_Motors[i]->Voltage() << 8) & 0xff00;
				m_Motors[i]->UnLock();
			}
		}
	}

	auto Send(const PackModel& refModel) -> void
	{
		if (m_Device)
		{
			m_Device->WriteRaw(sizeof(PackModel), reinterpret_cast<const uint8_t*>(&refModel));
		}
	}

	auto PackAndSend() -> void override
	{
		PackModel buffer{};
		Pack(buffer);
		Send(buffer);
	}

	auto CANId() const noexcept -> unsigned int override
	{
		return m_CANId;
	}

	auto Location() const -> std::string override
	{
		if (!m_Device) { throw std::runtime_error("Empty device"); }
		return m_Device->Bus()->Location();
	}

private:

	auto SetDevice(const std::shared_ptr<CANDevice> device) noexcept -> void override
	{
		m_Device = device;
	}

	Mutex m_Mutex;
	const unsigned int m_MotorsNum = 4;
	std::vector<std::shared_ptr<IMotor>> m_Motors;
	unsigned int m_CANId;
	std::shared_ptr<CANDevice> m_Device;
};

template <typename ReceiveModel,
	size_t CANIdBase,
	typename Mutex = std::mutex>
	class DJIMotor : public IMotor
{
	friend class MotorManager;
public:
	DJIMotor(unsigned int motorId)
		: m_MotorId(motorId)
	{
	}

	auto Get() noexcept -> ReceiveModel
	{
		std::lock_guard<Mutex> guard{m_Mutex};
		return m_Status;
	}

	auto GetRef() const noexcept -> const ReceiveModel& { return m_Status; }

	auto Lock() -> void override { m_Mutex.lock(); }

	auto UnLock() -> void override { m_Mutex.unlock(); }

	auto TryLock() noexcept -> bool override { return m_Mutex.try_lock(); }

	auto MotorId() const noexcept -> uint32_t override { return m_MotorId; }

	auto CANId() const noexcept -> unsigned int override { return m_MotorId + CANIdBase; }

	auto TimeStamp() const noexcept -> std::chrono::high_resolution_clock::time_point override { return m_TimeStamp; }

	auto Voltage() const noexcept -> int16_t override { return m_Voltage; }

	auto SetVoltage(const int16_t voltage) noexcept -> void override { m_Voltage = voltage; }

	auto Writer() const noexcept -> IMotorWriter* override { return m_Writer; }

private:
	Mutex m_Mutex;
	ReceiveModel m_Status = {};
	std::chrono::high_resolution_clock::time_point m_TimeStamp;
	unsigned int m_MotorId;
	uint16_t m_Voltage = 0;
	IMotorWriter* m_Writer;

	auto SetMotorId(const unsigned int id) noexcept -> void override
	{
		m_MotorId = id;
	}

	auto SetWriter(IMotorWriter* writer) -> void override
	{
		m_Writer = writer;
	}

	auto Parse(const uint8_t* buffer, const size_t bufferSize) -> void override
	{
		m_TimeStamp = std::chrono::high_resolution_clock::now();
		ReceiveModel::Parse(m_Status, buffer, bufferSize);
	}
};

template <typename Mutex = std::mutex>
using DJIMotor3508Writer = DJIMotorWriter<0x200, Mutex>;
template <typename Mutex = std::mutex>
using DJIMotor3508 = DJIMotor<ReceiveModel3508, 0x200, Mutex>;

using DJIMotor3508WriterMt = DJIMotor3508Writer<std::mutex>;
using DJIMotor3508WriterSt = DJIMotor3508Writer<null_mutex>;
using DJIMotor3508Mt = DJIMotor3508<std::mutex>;
using DJIMotor3508St = DJIMotor3508<null_mutex>;


template <typename Mutex = std::mutex>
using DJIMotor6020Writer = DJIMotorWriter<0x1ff, Mutex>;
template <typename Mutex = std::mutex>
using DJIMotor6020 = DJIMotor<ReceiveModel3508, 0x204, Mutex>;

using DJIMotor6020WriterMt = DJIMotor6020Writer<std::mutex>;
using DJIMotor6020WriterSt = DJIMotor6020Writer<null_mutex>;
using DJIMotor6020Mt = DJIMotor6020<std::mutex>;
using DJIMotor6020St = DJIMotor6020<null_mutex>;


template <typename Mutex = std::mutex>
using DJIMotor2006Writer = DJIMotorWriter<0x200, Mutex>;
template <typename Mutex = std::mutex>
using DJIMotor2006 = DJIMotor<ReceiveModel2006, 0x200, Mutex>;

using DJIMotor2006WriterMt = DJIMotor2006Writer<std::mutex>;
using DJIMotor2006WriterSt = DJIMotor2006Writer<null_mutex>;
using DJIMotor2006Mt = DJIMotor2006<std::mutex>;
using DJIMotor2006St = DJIMotor2006<null_mutex>;

} // ossian

#endif // OSSIAN_CORE_MOTORS_DJI_MOTOR