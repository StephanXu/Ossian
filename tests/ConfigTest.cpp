#include <spdlog/spdlog.h>
#include <Config.schema.hpp>
#include <ossian/Configuration.hpp>

int main()
{
	spdlog::info("This is a test for configuration module.");
	ossian::Utils::ConfigLoader<OssianConfig::configSchema> configLoader;
	
	std::string demoId{ "5f8c2c540553361014548e5f" };
	configLoader.LoadConfigFromUrl("ossian.mrxzh.com", 5000, fmt::format("/api/argument/{}?pt=true", demoId));
	spdlog::info("configLoader.Instance()->GetConnection()->GetSerialPort()->GetPortName() = {}",
		*configLoader.Instance().GetConnection()->GetSerialPort()->GetPortName());
}