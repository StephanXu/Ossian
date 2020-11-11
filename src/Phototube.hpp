#ifndef OSSIAN_PHOTOTUBE
#define OSSIAN_PHOTOTUBE

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>

struct PhototubeStatus
{
	enum StatusType
	{
		NO_BULLET = 0,
		HAS_BULLET = 1,
		UNSTABLE = 2
	};

	uint8_t m_Status;
	uint8_t m_Reserved;
	uint16_t m_AdValue;

	static auto Parse(PhototubeStatus& outModel,
	                  const uint8_t* buffer,
	                  const size_t bufferSize) -> void
	{
		if (!buffer)
		{
			memset(&outModel, 0, sizeof(PhototubeStatus));
			throw ossian::GeneralIOParseFailed{ "Phototube parse failed" };
		}

		StatusType status;
		AdValue adValue;
		adValue.m_Buf[0]    = buffer[2];
		adValue.m_Buf[1]    = buffer[3];
		outModel.m_Status   = buffer[0];
		outModel.m_Reserved = buffer[1];
		outModel.m_AdValue  = adValue.m_Value;
	}

private:
	union AdValue
	{
		uint16_t m_Value;
		unsigned char m_Buf[2];
	};
};

template <typename Mutex = std::mutex>
using Phototube = ossian::GeneralIO<PhototubeStatus, ossian::CANManager, Mutex>;

using PhototubeMt = Phototube<std::mutex>;
using PhototubeSt = Phototube<ossian::null_mutex>;

#endif // OSSIAN_PHOTOTUBE
