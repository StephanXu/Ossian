#ifndef OSSIAN_GYRO_6AXIS
#define OSSIAN_GYRO_6AXIS

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>


enum class GyroType
{
	Chassis,
	Gimbal
};

template <GyroType Gt>
struct GyroA110Status
{
	static constexpr auto GType() -> GyroType { return Gt; }

	double m_XAxisSpeed;
	double m_YAxisSpeed;
	double m_ZAxisSpeed;

	double m_XAngleSpeed;
	double m_YAngleSpeed;
	double m_ZAngleSpeed;

	double m_Pitch;
	double m_Roll;
	double m_Yaw;
};

template <typename Mutex, GyroType Gt>
class GyroA110 : public ossian::IODataBuilder<Mutex, GyroA110Status<Gt>>
{
	ossian::CANManager* m_CANManager;
	ossian::IOData<GyroA110Status<Gt>>* m_IOData;

public:
	OSSIAN_SERVICE_SETUP (GyroA110(ossian::CANManager* ioManager, ossian::IOData<GyroA110Status<Gt>>* ioData))
		: m_CANManager(ioManager)
		  , m_IOData(ioData)
	{
	}

	auto Add(const std::string location, const unsigned int speedId, const unsigned int angleId,
	         const unsigned int posId) -> void
	{
		auto speedProc = [this](const std::shared_ptr<ossian::BaseDevice>& device,
		                        const size_t length,
		                        const uint8_t* data)
		{
			if (length != 8)
			{
				return;
			}

			OriginalData xOriginal, yOriginal, zOriginal;
			xOriginal.m_Buf[0] = data[0];
			xOriginal.m_Buf[1] = data[1];
			yOriginal.m_Buf[0] = data[2];
			yOriginal.m_Buf[1] = data[3];
			zOriginal.m_Buf[0] = data[4];
			zOriginal.m_Buf[1] = data[5];

			auto model         = m_IOData->Get();
			model.m_XAxisSpeed = static_cast<double>(xOriginal.m_Value) * 0.001;
			model.m_YAxisSpeed = static_cast<double>(yOriginal.m_Value) * 0.001;
			model.m_ZAxisSpeed = static_cast<double>(zOriginal.m_Value) * 0.001;
			m_IOData->Set(model);
		};

		auto angleProc = [this](const std::shared_ptr<ossian::BaseDevice>& device,
		                        const size_t length,
		                        const uint8_t* data)
		{
			if (length != 8)
			{
				return;
			}

			OriginalData xOriginal, yOriginal, zOriginal;
			xOriginal.m_Buf[0] = data[0];
			xOriginal.m_Buf[1] = data[1];
			yOriginal.m_Buf[0] = data[2];
			yOriginal.m_Buf[1] = data[3];
			zOriginal.m_Buf[0] = data[4];
			zOriginal.m_Buf[1] = data[5];

			auto model          = m_IOData->Get();
			model.m_XAngleSpeed = static_cast<double>(xOriginal.m_Value) * 0.1;
			model.m_YAngleSpeed = static_cast<double>(yOriginal.m_Value) * 0.1;
			model.m_ZAngleSpeed = static_cast<double>(zOriginal.m_Value) * 0.1;
			m_IOData->Set(model);
		};

		auto posProc = [this](const std::shared_ptr<ossian::BaseDevice>& device,
		                      const size_t length,
		                      const uint8_t* data)
		{
			if (length != 8)
			{
				return;
			}

			OriginalData pitchOriginal, rollOriginal, yawOriginal;
			pitchOriginal.m_Buf[0] = data[0];
			pitchOriginal.m_Buf[1] = data[1];
			rollOriginal.m_Buf[0]  = data[2];
			rollOriginal.m_Buf[1]  = data[3];
			yawOriginal.m_Buf[0]   = data[4];
			yawOriginal.m_Buf[1]   = data[5];

			auto model    = m_IOData->Get();
			model.m_Pitch = static_cast<double>(pitchOriginal.m_Value) * 0.01;
			model.m_Roll  = static_cast<double>(rollOriginal.m_Value) * 0.01;
			model.m_Yaw   = static_cast<double>(yawOriginal.m_Value) * 0.1;
			m_IOData->Set(model);
		};

		m_CANManager->AddDevice(location, angleId)->SetCallback(angleProc);
		m_CANManager->AddDevice(location, speedId)->SetCallback(speedProc);
		m_CANManager->AddDevice(location, posId)->SetCallback(posProc);
	}

private:
	union OriginalData
	{
		int16_t m_Value;
		uint8_t m_Buf[2];
	};
};

template <GyroType Gt>
using GyroA110Mt = GyroA110<std::mutex, Gt>;

template <GyroType Gt>
using GyroA110St = GyroA110<ossian::null_mutex, Gt>;

#endif // OSSIAN_GYRO_6AXIS
