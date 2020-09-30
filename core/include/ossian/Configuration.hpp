#ifndef OSSIAN_CORE_CONFIGURATION
#define OSSIAN_CORE_CONFIGURATION

#include <spdlog/spdlog.h>

#include <string>
#include <variant>
#include <any>
#include <memory>

#include "Http.hpp"
#include "ApplicationBuilder.hpp"
#include <nlohmann/json.hpp>

namespace ossian
{
namespace Utils
{
class ConfigServiceBuilder;

class ConfigParseError : public std::runtime_error
{
public:
	ConfigParseError(std::string message): std::runtime_error(message)
	{
	}
};

/**
 * @class	Configuration
 *
 * @brief	配置文件解析
 *
 * @author	Xu Zihan, Huang Jiaxin
 * @date	2020/9/30
 */
class ConfigLoader : public CustomBuilder<ConfigServiceBuilder>
{
public:
	OSSIAN_SERVICE_SETUP(ConfigLoader())
	{
	}

	template <typename ConfigType>
	void LoadConfig(std::string text)
	{
		m_Config.reset();
		try
		{
			ConfigType config = nlohmann::json::parse(text);
			m_Config = config;
		}
		catch (std::exception& e)
		{
			throw ConfigParseError{ e.what() };
		}
		m_Valid = true;
	}

	template <typename ConfigType>
	ConfigType* Instance()
	{
		if (m_Valid)
			return std::any_cast<ConfigType>(&m_Config);
		return nullptr;
	}

	template <typename ConfigType>
	void LoadConfigFromFile(std::string configFilename)
	{
		std::ifstream f(configFilename);
		std::string text{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
		try
		{
			LoadConfig<ConfigType>(text);
		}
		catch (ConfigParseError& err)
		{
			SPDLOG_ERROR("Parse configuration from file {} failed: {}", configFilename, err.what());
			std::abort();
		}
		SPDLOG_TRACE("Load configuration from file: {}", configFilename);
	}

	template <typename ConfigType>
	void LoadConfigFromUrl(std::string host, int port, std::string path)
	{
		httplib::Client cli(host, port);
		auto res = cli.Get(path.c_str());
		if (!res || res->status != 200)
		{
			throw std::runtime_error(fmt::format("Fetch configuration from url failed: {}", res->status));
		}
		try
		{
			LoadConfig<ConfigType>(res->body);
		}
		catch (ConfigParseError& err)
		{
			SPDLOG_ERROR("Parse configuration from {} failed: {}", host, err.what());
			std::abort();
		}
		SPDLOG_TRACE("Load configuration from {}", host);
	}

	bool Valid() const noexcept
	{
		return m_Valid;
	}

private:
	bool m_Valid;
	std::any m_Config;
};

class ConfigServiceBuilder
{
	using BaseBuilder = BaseServiceBuilder<ConfigLoader>;
	ApplicationBuilder& m_AppBuilder;
	BaseBuilder& m_BaseBuilder;
public:
	ConfigServiceBuilder(ApplicationBuilder& appBuilder, BaseBuilder& config)
		: m_AppBuilder(appBuilder), m_BaseBuilder(config)
	{
	}

	template <typename ConfigType>
	auto LoadFromUrl(std::string host, int port, std::string path) -> ConfigServiceBuilder&
	{
		m_BaseBuilder.AddConfig([host, port, path](Utils::ConfigLoader& option)
		{
			option.LoadConfigFromUrl<ConfigType>(host, port, path);
		});
		return *this;
	}

	template <typename ConfigType>
	auto LoadFromFile(std::string filename) -> ConfigServiceBuilder&
	{
		m_BaseBuilder.AddConfig([filename](Utils::ConfigLoader& option)
		{
			option.LoadConfigFromFile<ConfigType>(filename);
		});
		return *this;
	}
};

} //Utils

} //ossian


#endif // OSSIAN_CORE_CONFIGURATION
