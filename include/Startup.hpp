#ifndef STARTUP_HPP
#define STARTUP_HPP

#include <ossian/ossian.hpp>

class Startup
{
    using AppBuilder = ossian::IOAP::ApplicationBuilder;

public:
    static void ConfigServices(AppBuilder& app);

    static void ConfigPipeline(AppBuilder& app);
};

#endif //STARTUP_HPP