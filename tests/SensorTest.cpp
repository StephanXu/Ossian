#include <ossian/io/CAN.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include "../src/GyroA204.hpp"
#include "../src/OnlineDebug.hpp"
#include <ossian/Utility.hpp>
#include "../src/Phototube.hpp"
#define NOTWITH_ONLINEDEBUG
std::chrono::time_point<std::chrono::system_clock> now, prev;


using SensorModel = PhototubeStatus;
int main()
{
#ifdef WITH_GPERF
    ProfilerStart("SensorTest.prof");
#endif
    const std::string location = "can0";
    const unsigned int deviceId = 0x300;
    const int epollTimeout = 1000;
    prev = std::chrono::system_clock::now();
#ifdef WITH_ONLINEDEBUG
    OnlineDebug onlineDbg;
    onlineDbg.Connect("http://ossian.mrxzh.com/logger");
    onlineDbg.StartLogging("OnlineLog",
                           "Sensor",
                           "Log of sensor");
    SPDLOG_TRACE("Start testing...");
#endif
    ossian::IOListener listener;
    auto mgr = std::make_shared<ossian::CANManager>(&listener);
    mgr->AddDevice(location, deviceId)
        ->SetCallback(
            [](std::shared_ptr<ossian::CANDevice> const& device, const size_t length, const uint8_t* data)
            {
                SensorModel model;
                SensorModel::Parse(model, data, length);
                SPDLOG_INFO("@Phototube=[$m_Status={},$m_AdValue={}]",
                            model.m_Status, model.m_AdValue);
            });
    auto bus = mgr->Bus(location);
    while (true)
    {
        bus->Read();
    }
#ifdef WITH_GPREF
    ProfilerStop();
#endif
}
