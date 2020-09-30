#ifndef STARTUP_HPP
#define STARTUP_HPP

#include <ossian/ApplicationBuilder.hpp>
#include <Config.schema.hpp>

class Startup : public ossian::IStartup
{
    using AppBuilder = ossian::ApplicationBuilder;
    OssianConfig::configSchema m_Config;
	
public:
    Startup();

    auto ConfigServices(AppBuilder& app) -> void override;

    auto ConfigPipeline(AppBuilder& app) -> void override;
};

#endif //STARTUP_HPP