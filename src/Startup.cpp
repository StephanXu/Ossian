#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"

class RoboStatus : public NautilusVision::IOAP::BaseStatus
{
};

void Startup::ConfigServices(AppBuilder &app)
{
    app.RegisterStatusType<RoboStatus>();
    app.RegisterService<VideoInputSource>();
}

void Startup::ConfigPipeline(AppBuilder &app)
{
    app.RegisterPipeline<RoboStatus,
                         ImageInputData,
                         WindmillDetection>();
    app.Register(CreateWindmillDetection);
}
