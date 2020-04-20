#ifndef STARTUP_HPP
#define STARTUP_HPP

#include <ossian/ApplicationBuilder.hpp>

#include "Config.pb.h"

class Startup : public ossian::IStartup
{
    using AppBuilder = ossian::ApplicationBuilder;
    OssianConfig::Configuration m_Config;
	
public:
    Startup();

    auto ConfigServices(AppBuilder& app) -> void override;

    auto ConfigPipeline(AppBuilder& app) -> void override;
};

#endif //STARTUP_HPP