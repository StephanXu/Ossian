#include <spdlog/spdlog.h>
#include <Config.schema.hpp>
#include <ossian/Configuration.hpp>

int main()
{
	spdlog::info("This is a test for configuration module.");
	ossian::Utils::ConfigLoader<Config::ConfigSchema> configLoader;

	std::string demoId{"5f9538c3f3ec8e31006b9001"};
	configLoader.LoadConfigFromUrl("lorime.club", demoId, 6203);
	spdlog::info("configLoader.Instance()->connection->serialPort->portName = {}",
	             *configLoader->connection->serialPort->portName);
}
