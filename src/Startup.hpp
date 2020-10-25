#ifndef STARTUP_HPP
#define STARTUP_HPP

#include <ossian/ApplicationBuilder.hpp>
#include <LaunchSettings.schema.hpp>

class Startup : public ossian::IStartup
{
    using AppBuilder = ossian::ApplicationBuilder;
    ossian::Utils::ConfigLoader<LaunchSettings::LaunchSettingsSchema> m_AppConfig;
	
public:
    Startup();

    auto ConfigServices(AppBuilder& app) -> void override;

    auto ConfigPipeline(AppBuilder& app) -> void override;
};

#endif //STARTUP_HPP