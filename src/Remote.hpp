/**
 * @file Remote.hpp
 * @author Xu Zihan (mrxzh@outlook.com)
 * @brief Remote controller I/O logic
 * @version 0.1
 * @date 2020-02-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_REMOTE_HPP
#define OSSIAN_REMOTE_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/MultiThread.hpp>
#include <ossian/IOData.hpp>
#include <ossian/GeneralIO.hpp>

#include <mutex>
#include <cstring>

/**
 * @brief Remote controller I/O data model
 */
struct RemoteStatus
{
	/* rocker channel information */
	int16_t ch[5];

	/* left and right lever information */
	uint8_t sw[2];

	static auto Parse(RemoteStatus& outModel,
	                  const uint8_t* buffer,
	                  const size_t bufferSize) -> void
	{
		const size_t packSize = 18;
		if (packSize > bufferSize)
		{
			memset(&outModel, 0, sizeof(RemoteStatus));
			throw ossian::GeneralIOParseFailed{ "Remote parse failed: packSize > bufferSize" };
		}
		outModel.ch[0] = (buffer[0] | buffer[1] << 8) & 0x07FF;
		outModel.ch[0] -= 1024;
		outModel.ch[1] = (buffer[1] >> 3 | buffer[2] << 5) & 0x07FF;
		outModel.ch[1] -= 1024;
		outModel.ch[2] = (buffer[2] >> 6 | buffer[3] << 2 | buffer[4] << 10) & 0x07FF;
		outModel.ch[2] -= 1024;
		outModel.ch[3] = (buffer[4] >> 1 | buffer[5] << 7) & 0x07FF;
		outModel.ch[3] -= 1024;

		outModel.ch[4] = (buffer[16] | buffer[17] << 8) & 0x7FF;
		outModel.ch[4] -= 1024;

		outModel.sw[0] = ((buffer[5] >> 4) & 0x000C) >> 2;
		outModel.sw[1] = (buffer[5] >> 4) & 0x0003;
		if ((abs(outModel.ch[0]) > 660) ||
		    (abs(outModel.ch[1]) > 660) ||
		    (abs(outModel.ch[2]) > 660) ||
		    (abs(outModel.ch[3]) > 660) ||
		    (abs(outModel.ch[4]) > 660) ||
		    outModel.sw[0] == 0 ||
		    outModel.sw[1] == 0)
		{
			memset(&outModel, 0, sizeof(RemoteStatus));
			throw ossian::GeneralIOParseFailed{ "Remote parse failed" };
		}
		SPDLOG_INFO("CH0:{} CH1:{} CH2:{} CH3:{} CH4:{} SW1:{} SW2:{}",
			outModel.ch[0], outModel.ch[1], outModel.ch[2], outModel.ch[3], outModel.ch[4],
			outModel.sw[0], outModel.sw[1]);
	}
};

template <typename Mutex>
using Remote = ossian::GeneralIO<RemoteStatus, ossian::UARTManager, Mutex>;

using RemoteMt = Remote<std::mutex>;
using RemoteSt = Remote<ossian::null_mutex>;

#endif // OSSIAN_REMOTE_HPP
