
#include "Startup.hpp"
#include "InputAdapter.hpp"
#include "WindmillDetection.hpp"
#include "Aimbot.hpp"

class RoboStatus : public NautilusVision::IOAP::BaseStatus
{
public:
};


std::unique_ptr<WindmillDetection> CreateWindmillDetection(SerialPortIO* serialPort)
{
    static ColorFilter redFilter{{{{170, 100, 100}, {180, 255, 255}},
                                  {{0, 100, 100}, {25, 255, 255}}}};
    static ColorFilter blueFilter{{{{85, 100, 100}, {135, 255, 255}}}};

	return std::make_unique<WindmillDetection>(80, redFilter, serialPort);
}

void Startup::ConfigServices(AppBuilder &app)
{
	app.InitLog();
	app.RegisterConfiguration();
    app.RegisterStatusType<RoboStatus>();
    app.RegisterInputAdapter<VideoInputSource>();
	app.RegisterService<SerialPortIO>();
}

void Startup::ConfigPipeline(AppBuilder& app)
{
	app.RegisterPipeline<RoboStatus, ImageInputData,
		WindmillDetection>();
	app.Register(CreateWindmillDetection);
}
