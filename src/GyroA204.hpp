#ifndef OSSIAN_GYRO_A204
#define OSSIAN_GYRO_A204

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>

struct GyroA204Status
{
	float m_ZAxis;
	uint16_t m_AdValue;
	uint8_t m_Constants[2];

	static auto Parse(GyroA204Status& outModel,
	                  const uint8_t* buffer,
	                  const size_t bufferSize) -> void
	{
		if (!buffer)
		{
			return;
		}

		ZRotateAngle zRotateAngle;
		AdValue adValue;

		zRotateAngle.m_Buf[0]   = buffer[0];
		zRotateAngle.m_Buf[1]   = buffer[1];
		zRotateAngle.m_Buf[2]   = buffer[2];
		zRotateAngle.m_Buf[3]   = buffer[3];
		adValue.m_Buf[0]        = buffer[4];
		adValue.m_Buf[1]        = buffer[5];
		outModel.m_Constants[0] = buffer[6];
		outModel.m_Constants[1] = buffer[7];
		outModel.m_ZAxis        = zRotateAngle.m_Value;
		outModel.m_AdValue      = adValue.m_Value;
	}

private:
	union AdValue
	{
		uint16_t m_Value;
		unsigned char m_Buf[2];
	};

	union ZRotateAngle
	{
		float m_Value;
		unsigned char m_Buf[4];
	};
};

template <typename Mutex=std::mutex>
using GyroA204 = ossian::GeneralIO<GyroA204Status, ossian::CANManager, Mutex>;

using GyroA204Mt = GyroA204<std::mutex>;
using GyroA204St = GyroA204<ossian::null_mutex>;

#endif // OSSIAN_GYRO_A204
