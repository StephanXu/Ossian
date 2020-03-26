#ifndef OSSIAN_GYRO_HPP
#define OSSIAN_GYRO_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/IOData.hpp>

#include <mutex>
#include "ossian/ApplicationBuilder.hpp"

struct GyroModel
{
	double m_Ax, m_Ay, m_Az;
	double m_Wx, m_Wy, m_Wz;
	double m_Roll, m_Pitch, m_Yaw;
	double m_Hx, m_Hy, m_Hz;
};

class IGyro
{
public:
	virtual ~IGyro() = default;
	virtual auto AddGyro(const std::string location, const unsigned int id) -> void = 0;
};

class Gyro : public IGyro
{
public:
	OSSIAN_SERVICE_SETUP(Gyro(ossian::CANManager* canManager,
		ossian::IOData<GyroModel>* dataListener))
		: m_CANManager(canManager)
		  , m_DataListener(dataListener)
	{
	}

	auto AddGyro(const std::string location, const unsigned int id) -> void override
	{
		if (!m_CANManager)
		{
			throw std::runtime_error("CANManager is null");
		}
		m_CANManager->AddDevice(location, id)->SetCallback(
			[this](const std::shared_ptr<ossian::BaseDevice>& device, const size_t length,
			       const uint8_t* data)
			{
				const double g = 9.8;
				if (0x55 != data[0] || 8 != length) { return; }
				auto model = m_DataListener->Get();
				switch (data[1])
				{
				case 0x51: ///< 加速度
					model.m_Ax = ((data[3] << 8) | data[2]) / 32768 * 16 * g;
					model.m_Ay = ((data[5] << 8) | data[4]) / 32768 * 16 * g;
					model.m_Az = ((data[7] << 8) | data[6]) / 32768 * 16 * g;
					break;
				case 0x52: ///< 角速度
					model.m_Wx = ((data[3] << 8) | data[2]) / 32768 * 2000;
					model.m_Wy = ((data[5] << 8) | data[4]) / 32768 * 2000;
					model.m_Wz = ((data[7] << 8) | data[6]) / 32768 * 2000;
					break;
				case 0x53: ///< 角度
					model.m_Roll = ((data[3] << 8) | data[2]) / 32768 * 180;
					model.m_Pitch = ((data[5] << 8) | data[4]) / 32768 * 180;
					model.m_Yaw   = ((data[7] << 8) | data[6]) / 32768 * 180;
					break;
				case 0x54: ///< 磁场
					model.m_Hx = ((data[3] << 8) | data[2]);
					model.m_Hy = ((data[5] << 8) | data[4]);
					model.m_Hz = ((data[7] << 8) | data[6]);
					break;
				}
				m_DataListener->Set(model);
			});
	}

private:
	ossian::CANManager* m_CANManager;
	ossian::IOData<GyroModel>* m_DataListener;
};

namespace ossian
{
template <>
class ServiceBuilder<Gyro> : BaseServiceBuilder<Gyro>
{
	using ServiceType = Gyro;
	using Super = BaseServiceBuilder<ServiceType>;
public:
	ServiceBuilder(ApplicationBuilder& appBuilder,
	               std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<ServiceType>(appBuilder, configureProc)
	{
		Super::m_AppBuilder.template AddService<IOData<GyroModel>>();
	}
};
} // ossian

#endif // OSSIAN_GYRO_HPP
