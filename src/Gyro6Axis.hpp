#ifndef OSSIAN_GYRO_6AXIS
#define OSSIAN_GYRO_6AXIS

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>

struct Gyro6AxisStatus
{
	int16_t m_XAxisSpeed;
	int16_t m_YAxisSpeed;
	int16_t m_ZAxisSpeed;
	
	int16_t m_XAngleSpeed;
	int16_t m_YAngleSpeed;
	int16_t m_ZAngleSpeed;

	int16_t m_Pitch;
	int16_t m_Roll;
	int16_t m_Yaw;
};

template <typename Mutex>
class Gyro6Axis : public ossian::IODataBuilder<Mutex, Gyro6AxisStatus>
{
	ossian::CANManager* m_CANManager;
	ossian::IOData<Gyro6AxisStatus>* m_IOData;

public:
	OSSIAN_SERVICE_SETUP(Gyro6Axis(ossian::CANManager* ioManager, ossian::IOData<Gyro6AxisStatus>* ioData))
		: m_CANManager(ioManager)
		, m_IOData(ioData)
	{}

	auto Add(const std::string location, const unsigned int speedId, const unsigned int angleId, const unsigned int posId) -> void
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
			
			auto model = m_IOData->Get();
			model.m_XAxisSpeed = xOriginal.m_Value;
			model.m_YAxisSpeed = yOriginal.m_Value;
			model.m_ZAxisSpeed = zOriginal.m_Value;
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

			auto model = m_IOData->Get();
			model.m_XAngleSpeed = xOriginal.m_Value;
			model.m_YAngleSpeed = yOriginal.m_Value;
			model.m_ZAngleSpeed = zOriginal.m_Value;
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
			rollOriginal.m_Buf[0] = data[2];
			rollOriginal.m_Buf[1] = data[3];
			yawOriginal.m_Buf[0] = data[4];
			yawOriginal.m_Buf[1] = data[5];

			auto model = m_IOData->Get();
			model.m_XAngleSpeed = pitchOriginal.m_Value;
			model.m_YAngleSpeed = rollOriginal.m_Value;
			model.m_ZAngleSpeed = yawOriginal.m_Value;
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

using Gyro6AxisMt = Gyro6Axis<std::mutex>;

using Gyro6AxisSt = Gyro6Axis<ossian::null_mutex>;

#endif // OSSIAN_GYRO_6AXIS
