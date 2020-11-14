#ifndef OSSIAN_CAPACITOR_HPP
#define OSSIAN_CAPACITOR_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/MultiThread.hpp>
#include <ossian/IOData.hpp>

#include <mutex>

struct CapacitorStatus
{
	float m_InputVoltage;
	float m_CapacitorVoltage;
	float m_TestCurrent;
	float m_TargetPower;
};

class ICapacitor
{
public:
	virtual auto AddCapacitor(const std::string location, const unsigned int readerId, const unsigned int writerId)->void = 0;

	/**
	 * @brief 设置功率
	 * @param power 功率值(W)
	 */
	virtual auto SetPower(const double power)->void = 0;
};

template<typename Mutex = std::mutex>
class Capacitor : ossian::IODataBuilder<Mutex, CapacitorStatus>
{
#pragma pack(push,1)
	struct SetPowerModel
	{
		uint16_t m_Power;
	};

	struct StatusModel
	{
		uint16_t m_InputVoltage;
		uint16_t m_CapacitorVoltage;
		uint16_t m_TestCurrent;
		uint16_t m_TargetPower;
	};
#pragma pack(pop)	
	ossian::IOData<CapacitorStatus>* m_Status{};
	ossian::CANManager* m_CANManager;
	std::shared_ptr<ossian::CANDevice> m_WriterDevice{};

public:
	OSSIAN_SERVICE_SETUP(Capacitor(
		ossian::CANManager* canManager, 
		ossian::IOData<CapacitorStatus>* capacitorStatus))
		:m_CANManager(canManager)
		,m_Status(capacitorStatus)
	{
	}

	auto AddCapacitor(const std::string location,
					  const unsigned int readerId,
					  const unsigned writerId)->void 
	{
		if (!m_CANManager)
		{
			throw std::runtime_error("CANManager is null");
		}
		m_CANManager->AddDevice(location, readerId)->SetCallback(
			[this](const std::shared_ptr<ossian::BaseDevice>& device,
				   const size_t length,
				   const uint8_t* data)
			{
				CapacitorStatus status;
				auto model = reinterpret_cast<const StatusModel*>(data);
				status.m_InputVoltage = model->m_InputVoltage / 100.f;
				status.m_CapacitorVoltage = model->m_CapacitorVoltage / 100.f;
				status.m_TestCurrent = model->m_TestCurrent / 100.f;
				status.m_TargetPower = model->m_TargetPower / 100.f;
				m_Status->Set(status);
			});
		m_WriterDevice = m_CANManager->AddDevice(location, writerId);
	}

	auto SetPower(const double power)->void 
	{
		SetPowerModel data{ static_cast<uint16_t>(power * 100) };
		m_WriterDevice->WriteRaw(sizeof(SetPowerModel), reinterpret_cast<uint8_t*>(&data));
	}
};

using CapacitorMt = Capacitor<std::mutex>;
using CapacitorSt = Capacitor<ossian::null_mutex>;

#endif // OSSIAN_CAPACITOR_HPP