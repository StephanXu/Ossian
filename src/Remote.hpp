/**
 * @file Remote.hpp
 * @author Xu Zihan (im.xuzihan@outlook.com)
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

#include <map>
#include <mutex>
#include <cstring>

constexpr uint8_t kRCSwUp   = 1;
constexpr uint8_t kRCSwMid  = 3;
constexpr uint8_t kRCSwDown = 2;

constexpr uint16_t kKeyPressedOffsetW		= 1 << 0;
constexpr uint16_t kKeyPressedOffsetS		= 1 << 1;
constexpr uint16_t kKeyPressedOffsetA		= 1 << 2;
constexpr uint16_t kKeyPressedOffsetD		= 1 << 3;
constexpr uint16_t kKeyPressedOffsetShift	= 1 << 4;
constexpr uint16_t kKeyPressedOffsetCtrl	= 1 << 5;
constexpr uint16_t kKeyPressedOffsetQ		= 1 << 6;
constexpr uint16_t kKeyPressedOffsetE		= 1 << 7;
constexpr uint16_t kKeyPressedOffsetR		= 1 << 8;
constexpr uint16_t kKeyPressedOffsetF		= 1 << 9;
constexpr uint16_t kKeyPressedOffsetG		= 1 << 10;
constexpr uint16_t kKeyPressedOffsetZ		= 1 << 11;
constexpr uint16_t kKeyPressedOffsetX		= 1 << 12;
constexpr uint16_t kKeyPressedOffsetC		= 1 << 13;
constexpr uint16_t kKeyPressedOffsetV		= 1 << 14;
constexpr uint16_t kKeyPressedOffsetB		= 1 << 15;

const std::map<std::string, uint16_t> kKeyboardMapChassis{
	{"Forward", kKeyPressedOffsetW},
	{"Backward", kKeyPressedOffsetS},
	{"Leftward", kKeyPressedOffsetA},
	{"Rightward", kKeyPressedOffsetD},
	{"TopMode", kKeyPressedOffsetE}
};

const std::map<std::string, uint16_t> kKeyboardMapGimbal;
const std::map<std::string, uint16_t> kKeyboardMapFric{
	{"FricOn", kKeyPressedOffsetF}
};
const std::map<std::string, uint16_t> kKeyboardMapFeed;

 /**
  * @brief Remote controller I/O data model
  */
struct RemoteStatus
{
	/* rocker channel information */
	int16_t ch[5];

	/* left and right lever information */
	uint8_t sw[2];

	/* 鼠标 mouse[x,y,z] click[left,right] */
	int16_t mouse[3];
	bool click[2];

	/* 键盘 */
	uint16_t keyboard;

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

		outModel.mouse[0] = buffer[6] | buffer[7] << 8;
		outModel.mouse[1] = buffer[8] | buffer[9] << 8;
		outModel.mouse[2] = buffer[10] | buffer[11] << 8;
		outModel.click[0] = buffer[12];
		outModel.click[1] = buffer[13];

		outModel.keyboard = buffer[14] | buffer[15] << 8;

		if ((abs(outModel.ch[0]) > 660) ||
			(abs(outModel.ch[1]) > 660) ||
			(abs(outModel.ch[2]) > 660) ||
			(abs(outModel.ch[3]) > 660) ||
			(abs(outModel.ch[4]) > 660) ||
			outModel.sw[0] == 0 ||
			outModel.sw[1] == 0)
		{
			memset(&outModel, 0, sizeof(RemoteStatus));
			SPDLOG_WARN("CH0:{} CH1:{} CH2:{} CH3:{} CH4:{} SW1:{} SW2:{}",
				outModel.ch[0], outModel.ch[1], outModel.ch[2], outModel.ch[3], outModel.ch[4],
				outModel.sw[0], outModel.sw[1]);
			SPDLOG_WARN("MOUSEX:{} MOUSEY:{} MOUSEZ:{} CLICKL:{} CLICKR:{} KEYBOARD:{}",
				outModel.mouse[0], outModel.mouse[1], outModel.mouse[2], outModel.click[0], outModel.click[1],
				outModel.keyboard);
			throw ossian::GeneralIOParseFailed{ "Remote parse failed" };
		}
		SPDLOG_TRACE("CH0:{} CH1:{} CH2:{} CH3:{} CH4:{} SW1:{} SW2:{}",
			outModel.ch[0], outModel.ch[1], outModel.ch[2], outModel.ch[3], outModel.ch[4],
			outModel.sw[0], outModel.sw[1]);
		SPDLOG_TRACE("MOUSEX:{} MOUSEY:{} MOUSEZ:{} CLICKL:{} CLICKR:{} KEYBOARD:{}",
			outModel.mouse[0], outModel.mouse[1], outModel.mouse[2], outModel.click[0], outModel.click[1],
			outModel.keyboard);
	}
};

template <typename Mutex>
using Remote = ossian::GeneralIO<RemoteStatus, ossian::UARTManager, Mutex>;

using RemoteMt = Remote<std::mutex>;
using RemoteSt = Remote<ossian::null_mutex>;

#endif // OSSIAN_REMOTE_HPP
