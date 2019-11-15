#ifndef STARTUP_HPP
#define STARTUP_HPP

#include <nv/nv.hpp>

class Startup
{
    using AppBuilder = NautilusVision::IOAP::ApplicationBuilder;

public:
    static void ConfigServices(AppBuilder& app);

    static void ConfigPipeline(AppBuilder& app);
};

#endif //STARTUP_HPP