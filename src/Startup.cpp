
#include <ossian/ossian.hpp>
#include <mimalloc.h>
#include "Config.pb.h"

#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"
#include "SerialReport.hpp"

#include <thread>

class RoboStatus : public ossian::IOAP::BaseStatus
{
public:
    OSSIAN_SERVICE_SETUP(RoboStatus())
    {
    }
};

void Startup::ConfigServices(AppBuilder& app)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    app.InitLog();
    spdlog::info("MI_VERSION:{}", mi_version());

    app.AddService<Utils::ConfigLoader>().LoadFromUrl<OssianConfig::Configuration>("mrxzh.com", 5000, "/config");
    app.AddService<RoboStatus>();
    app.AddService<VideoInputSource>().AsInputAdapter();
    app.AddService<SerialPortIO>();
    //app.AddInputAdapter<CameraInputSource>();
    //app.AddInputAdapter<FakeInputSource>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
    //app.RegisterPipeline<RoboStatus, FakeData,
    //	SerialReport>();

    app.RegisterPipeline<RoboStatus, ImageInputData,
                         WindmillDetection>();
    app.Add(WindmillDetection::CreateWindmillDetection);

    //app.Register(Aimbot::CreateAimbot);
    //app.Register(SerialReport::CreateSerialReport);
}
