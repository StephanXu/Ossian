#include <spdlog/spdlog.h>
#include <Config.schema.hpp>
#include <ossian/Configuration.hpp>

int main()
{
	spdlog::info("This is a test for configuration module.");
	ossian::Utils::ConfigLoader configLoader;
	configLoader.LoadConfigFromFile<OssianConfig::configSchema>("/home/Ossian/Config.json");
	auto config = configLoader.Instance<OssianConfig::configSchema>();
	spdlog::info("*config->GetConnection()->GetCan()->GetCanName() = {}", *config->GetConnection()->GetCan()->GetCanName());
}