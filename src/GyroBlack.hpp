#ifndef OSSIAN_GYRO_BLACK
#define OSSIAN_GYRO_BLACK

#include <ossian/MultiThread.hpp>
#include <ossian/io/CAN.hpp>
#include <ossian/GeneralIO.hpp>

enum class GyroTypeBlack
{
    Pitch,
    Yaw
};

template <GyroTypeBlack Gt>
struct GyroBlackStatus
{
    static constexpr auto GType() -> GyroTypeBlack { return Gt; }
    float m_Angle;
};

template <typename Mutex, GyroTypeBlack Gt>
class GyroBlack : public ossian::IODataBuilder<Mutex, GyroBlackStatus<Gt>>
{
    ossian::CANManager* m_CANManager;
    ossian::IOData<GyroBlackStatus<Gt>>* m_IOData;

public:
    OSSIAN_SERVICE_SETUP(GyroBlack(ossian::CANManager* ioManager, ossian::IOData<GyroBlackStatus<Gt>>* ioData))
    : m_CANManager(ioManager)
    , m_IOData(ioData)
    {
    }

    auto Add(const std::string location, const unsigned int id = 403) -> void
    {
        auto proc = [this](const std::shared_ptr<ossian::BaseDevice>& device,
                                const size_t length,
                                const uint8_t* data)
        {
            if (length != 4)
            {
                return;
            }
            auto model         = m_IOData->Get();
            model.m_Angle = (float)(0.01)*((int32_t)(data[0]<<24)|(int32_t)(data[1]<<16) | (int32_t)(data[2]<<8) | (int32_t)(data[3]));
            m_IOData->Set(model);
        };

        m_CANManager->AddDevice(location, id)->SetCallback(proc);
    }

    auto Calibrate(const std::string location, const unsigned int id = 406) -> void
    {
        const uint8_t data[]={0,1,2,3,4,5,6,7};
        m_CANManager->AddDevice(location, id)->WriteRaw(sizeof(data), data);
    }
};

template <GyroTypeBlack Gt>
using GyroBlackMt = GyroBlack<std::mutex, Gt>;

template <GyroTypeBlack Gt>
using GyroBlackSt = GyroBlack<ossian::null_mutex, Gt>;

#endif // OSSIAN_GYRO_BLACK
