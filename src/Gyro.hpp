/**
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
		                                    ossian::UARTProperties::R115200,
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
				const size_t packSize{8};
				const double g{9.8};
				bool receiveFlag[4]{false, false, false, false};
				auto model = m_DataListener->Get();
				for (const uint8_t* readPtr{data + length - packSize}; readPtr - data >= 0; --
				     readPtr)
				{
					if (receiveFlag[0] && receiveFlag[1] && receiveFlag[2] && receiveFlag[3])
					{
						// All data have already updated.
						break;
					}
					if (readPtr[0] != 0x55)
					{
						// Looking for frame header.
						continue;
					}
					int16_t payloadValue[3] =
					{
						*reinterpret_cast<const int16_t*>(&readPtr[2]),
						*reinterpret_cast<const int16_t*>(&readPtr[4]),
						*reinterpret_cast<const int16_t*>(&readPtr[6])
					};
					switch (readPtr[1])
					{
					case 0x51: ///< 加速度
						if (receiveFlag[0])
						{
							break;
						}
						model.m_Ax     = payloadValue[0] / 32768.0 * 16.0 * g;
						model.m_Ay     = payloadValue[1] / 32768.0 * 16.0 * g;
						model.m_Az     = payloadValue[2] / 32768.0 * 16.0 * g;
						receiveFlag[0] = true;
						break;
					case 0x52: ///< 角速度
						if (receiveFlag[1])
						{
							break;
						}
						model.m_Wx     = payloadValue[0] / 32768.0 * 2000.0 / 180.0 * M_PI;
						model.m_Wy     = payloadValue[1] / 32768.0 * 2000.0 / 180.0 * M_PI;
						model.m_Wz     = payloadValue[2] / 32768.0 * 2000.0 / 180.0 * M_PI;
						receiveFlag[1] = true;
						break;
					case 0x53: ///< 角度
						if (receiveFlag[2])
						{
							break;
						}
						model.m_Roll   = payloadValue[0] / 32768.0 * M_PI;
						model.m_Pitch  = payloadValue[1] / 32768.0 * M_PI;
						model.m_Yaw    = payloadValue[2] / 32768.0 * M_PI;
						receiveFlag[2] = true;
						break;
					case 0x54: ///< 磁场
						if (receiveFlag[3])
						{
							break;
						}
						model.m_Hx     = payloadValue[0];
						model.m_Hy     = payloadValue[1];
						model.m_Hz     = payloadValue[2];
						receiveFlag[3] = true;
						break;
					}
					readPtr -= packSize - 1; ///< There is a extra minus after each loop.
				}
				m_DataListener->Set(model);
			});
		const uint8_t command[] = {'A', 'T', '+', 'E', 'T', '\n'};
		dev->WriteRaw(6, command);
	}

private:
	ossian::UARTManager* m_UARTManager;
	ossian::IOData<GyroModel>* m_DataListener;
};

using GyroMt = Gyro<std::mutex>;
using GyroSt = Gyro<ossian::null_mutex>;

#endif // OSSIAN_GYRO_HPP
