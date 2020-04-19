﻿/**
 * @file Gyro.hpp
 * @author Xu Zihan (mrxzh@outlook.com)
 * @brief Gyro I/O logic
 * @version 0.1
 * @date 2020-03-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_GYRO_HPP
#define OSSIAN_GYRO_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/IOData.hpp>

#include <mutex>
#include <sstream>
#include "ossian/ApplicationBuilder.hpp"

struct GyroModel
{
	double m_Ax, m_Ay, m_Az;
	double m_Wx, m_Wy, m_Wz;
	double m_Roll, m_Pitch, m_Yaw;
	double m_Hx, m_Hy, m_Hz;
};

/**
 * @brief Interface of gyro service.
 */
class IGyro
{
public:
	virtual ~IGyro() = default;

	/**
	 * @brief Initialize gyro.
	 * 
	 * @param location The location of gyro. (e.g. /dev/ttyTHS2)
	 */
	virtual auto AddGyro(const std::string location) -> void = 0;
};

template <typename Mutex = std::mutex>
class Gyro : public IGyro, public ossian::IODataBuilder<Mutex, GyroModel>
{
public:
	OSSIAN_SERVICE_SETUP(Gyro(ossian::UARTManager* uartManager,
		ossian::IOData<GyroModel>* dataListener))
		: m_UARTManager(uartManager)
		  , m_DataListener(dataListener)
	{
	}

	auto AddGyro(const std::string location) -> void override
	{
		if (!m_UARTManager)
		{
			throw std::runtime_error("UARTManager is null");
		}
		auto dev = m_UARTManager->AddDevice(location,
		                         ossian::UARTProperties::R230400,
		                         ossian::UARTProperties::FlowControlNone,
		                         ossian::UARTProperties::DataBits8,
		                         ossian::UARTProperties::StopBits1,
		                         ossian::UARTProperties::ParityNone)->SetCallback(
			[this](const std::shared_ptr<ossian::BaseDevice>& device, const size_t length,
			       const uint8_t* data)
			{
				std::stringstream ss;
				for (size_t i{}; i < length; ++i)
				{
					ss << fmt::format("{:02x}", data[i]);
					ss << " ";
				}
				spdlog::info("Gyro Buffer: len={} data={}", length, ss.str());
				spdlog::trace("Gyro Receive: {}, buffer: {}", length, data);
				const size_t packSize{ 8 };
				const double g{ 9.8 };
				auto model = m_DataListener->Get();
				for (const uint8_t* readPtr{ data }; readPtr - data < length; ++readPtr)
				{
					if ((data + length) - readPtr - 1 < packSize)
					{
						spdlog::warn("Gyro: Incomplete data.");
						return;
					}
					if (readPtr[0] != 0x55)
					{
						continue;
					}
					switch (readPtr[1])
					{
					case 0x51: ///< 加速度
						model.m_Ax = static_cast<double>((readPtr[3] << 8) | readPtr[2]) / 32768 * 16 * g;
						model.m_Ay = static_cast<double>((readPtr[5] << 8) | readPtr[4]) / 32768 * 16 * g;
						model.m_Az = static_cast<double>((readPtr[7] << 8) | readPtr[6]) / 32768 * 16 * g;
						break;
					case 0x52: ///< 角速度
						model.m_Wx = static_cast<double>((readPtr[3] << 8) | readPtr[2]) / 32768 * 2000 / 180 * M_PI;
						model.m_Wy = static_cast<double>((readPtr[5] << 8) | readPtr[4]) / 32768 * 2000 / 180 * M_PI;
						model.m_Wz = static_cast<double>((readPtr[7] << 8) | readPtr[6]) / 32768 * 2000 / 180 * M_PI;
						break;
					case 0x53: ///< 角度
						model.m_Roll = static_cast<double>((readPtr[3] << 8) | readPtr[2]) / 32768 * M_PI;
						model.m_Pitch = static_cast<double>((readPtr[5] << 8) | readPtr[4]) / 32768 * M_PI;
						model.m_Yaw = static_cast<double>((readPtr[7] << 8) | readPtr[6]) / 32768 * M_PI;
						break;
					case 0x54: ///< 磁场
						model.m_Hx = static_cast<double>((readPtr[3] << 8) | readPtr[2]);
						model.m_Hy = static_cast<double>((readPtr[5] << 8) | readPtr[4]);
						model.m_Hz = static_cast<double>((readPtr[7] << 8) | readPtr[6]);
						break;
					}
					readPtr += packSize;
					m_DataListener->Set(model);
				}
				
			});
		const uint8_t command[] = { 'A','T','+','E','T','\n' };
		dev->WriteRaw(6, command);
	}

private:
	ossian::UARTManager* m_UARTManager;
	ossian::IOData<GyroModel>* m_DataListener;
};

using GyroMt = Gyro<std::mutex>;
using GyroSt = Gyro<ossian::null_mutex>;

#endif // OSSIAN_GYRO_HPP
