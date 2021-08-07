#ifndef OSSIAN_TOF_NOOPLOOP
#define OSSIAN_TOF_NOOPLOOP

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>

struct NooploopTOFStatus
{
	uint32_t m_Distance;
	uint8_t m_DistanceStatus;
	uint16_t m_SignalStrength;
	uint8_t m_Reserved[2];

	static auto Parse(NooploopTOFStatus& outModel,
					  const uint8_t* buffer,
					  const size_t bufferSize) -> void
	{
		if (!buffer)
		{
			return;
		}

		Distance distance;
		DistanceStatus distanceStatus;
		SignalStrength signalStrength;

		distance.m_Buf[0] = buffer[0];
		distance.m_Buf[1] = buffer[1];
		distance.m_Buf[2] = buffer[2];
		distance.m_Buf[3] = 0;
		distanceStatus.m_Buf[0] = buffer[3];
		signalStrength.m_Buf[0] = buffer[4];
		signalStrength.m_Buf[1] = buffer[5];
		outModel.m_Reserved[0] = buffer[6];
		outModel.m_Reserved[1] = buffer[7];
		outModel.m_Distance = distance.m_Value;
		outModel.m_DistanceStatus = distanceStatus.m_Value;
		outModel.m_SignalStrength = signalStrength.m_Value;
	}

private:
	union Distance
	{
		uint32_t m_Value;
		unsigned char m_Buf[4];
	};

	union DistanceStatus
	{
		uint8_t m_Value;
		unsigned char m_Buf[1];
	};

	union SignalStrength
	{
		uint16_t m_Value;
		unsigned char m_Buf[2];
	};
};

template <typename Mutex = std::mutex>
using NooploopTOF = ossian::GeneralIO<NooploopTOFStatus, ossian::CANManager, Mutex>;

using NooploopTOFMt = NooploopTOF<std::mutex>;
using NooploopTOFSt = NooploopTOF<ossian::null_mutex>;

#endif // OSSIAN_TOF_NOOPLOOP
