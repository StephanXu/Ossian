#ifndef OSSIAN_GYRO_A204
#define OSSIAN_GYRO_A204

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>

enum class GyroType
{
	Pitch,
	Yaw
};

template <GyroType Gt>
struct GyroA204Status
{
	static constexpr auto GType() -> GyroType { return Gt; }

	float m_ZAxisAngle;
	int16_t m_XAxisSpeed;
	int16_t m_YAxisSpeed;

	float m_ZAxisSpeed;
	uint16_t m_AdValue;
};

template <typename Mutex, GyroType Gt>
class GyroA204 : public ossian::IODataBuilder<Mutex, GyroA204Status<Gt>>
{
	ossian::CANManager* m_CANManager;
	ossian::IOData<GyroA204Status<Gt>>* m_IOData;

public:
	OSSIAN_SERVICE_SETUP(GyroA204(ossian::CANManager* ioManager, ossian::IOData<GyroA204Status<Gt>>* ioData))
		: m_CANManager(ioManager)
		  , m_IOData(ioData)
	{
	}

	auto Add(const std::string location, const unsigned int angleId, const unsigned int speedId) -> void
	{
		auto angleProc = [this](const std::shared_ptr<ossian::BaseDevice>& device,
		                        const size_t length,
		                        const uint8_t* data)
		{
			if (length != 8)
			{
				return;
			}

			AngleData zRotateAngle;
			AdValue adValue;

			zRotateAngle.m_Buf[0] = data[0];
			zRotateAngle.m_Buf[1] = data[1];
			zRotateAngle.m_Buf[2] = data[2];
			zRotateAngle.m_Buf[3] = data[3];
			adValue.m_Buf[0]      = data[4];
			adValue.m_Buf[1]      = data[5];

			m_IOData->Lock();
			auto& model        = m_IOData->GetRef();
			model.m_ZAxisAngle = zRotateAngle.m_Value;
			model.m_AdValue    = adValue.m_Value;
			m_IOData->UnLock();
		};

		auto speedProc = [this](const std::shared_ptr<ossian::BaseDevice>& device,
		                        const size_t length,
		                        const uint8_t* data)
		{
			if (length != 8)
			{
				return;
			}

			AngleData zRotateAngle;
			OriginalAxisData xOriginal, yOriginal;

			zRotateAngle.m_Buf[0] = data[0];
			zRotateAngle.m_Buf[1] = data[1];
			zRotateAngle.m_Buf[2] = data[2];
			zRotateAngle.m_Buf[3] = data[3];
			xOriginal.m_Buf[0]    = data[4];
			xOriginal.m_Buf[1]    = data[5];
			yOriginal.m_Buf[0]    = data[6];
			yOriginal.m_Buf[1]    = data[7];

			m_IOData->Lock();
			auto& model        = m_IOData->GetRef();
			model.m_ZAxisSpeed = zRotateAngle.m_Value;
			model.m_XAxisSpeed = xOriginal.m_Value;
			model.m_YAxisSpeed = yOriginal.m_Value;
			m_IOData->UnLock();
		};

		m_CANManager->AddDevice(location, angleId)->SetCallback(angleProc);
		m_CANManager->AddDevice(location, speedId)->SetCallback(speedProc);
	}

private:
	union OriginalAxisData
	{
		int16_t m_Value;
		uint8_t m_Buf[2];
	};

	union AdValue
	{
		uint16_t m_Value;
		uint8_t m_Buf[2];
	};

	union AngleData
	{
		float m_Value;
		uint8_t m_Buf[4];
	};
};

template<GyroType Gt>
using GyroA204Mt = GyroA204<std::mutex, Gt>;

template<GyroType Gt>
using GyroA204St = GyroA204<ossian::null_mutex, Gt>;

#endif // OSSIAN_GYRO_A204
