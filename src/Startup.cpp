#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"

class RoboStatus : public NautilusVision::IOAP::BaseStatus
{
};


std::unique_ptr<WindmillDetection> CreateWindmillDetection()
{
    static ColorFilter redFilter{{{{170, 100, 100}, {180, 255, 255}},
                                  {{0, 100, 100}, {25, 255, 255}}}};
    static ColorFilter blueFilter{{{{85, 100, 100}, {135, 255, 255}}}};

    return std::make_unique<WindmillDetection>(80, redFilter);
}


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
