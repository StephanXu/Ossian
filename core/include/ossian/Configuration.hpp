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
template <typename ConfigType>
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
template <typename ConfigType>
class ConfigLoader : public CustomBuilder<ConfigServiceBuilder<ConfigType>>
{
public:
	OSSIAN_SERVICE_SETUP(ConfigLoader())
	{
	}

	void LoadConfig(std::string text)
	{
		try
		{
			ConfigType config = nlohmann::json::parse(text);
			m_Config          = config;
		}
		catch (std::exception& e)
		{
			throw ConfigParseError{e.what()};
		}
		m_Valid = true;
	}

	const ConfigType* Instance() const
	{
		return &m_Config;
	}

	void LoadConfigFromFile(std::string configFilename)
	{
		std::ifstream f(configFilename);
		std::string text{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
		try
		{
			LoadConfig(text);
		}
		catch (ConfigParseError& err)
		{
			SPDLOG_ERROR("Parse configuration from file {} failed: {}", configFilename, err.what());
			std::abort();
		}
		SPDLOG_TRACE("Load configuration from file: {}", configFilename);
	}

	void LoadConfigFromUrl(const std::string host, const std::string id, const unsigned int port = 80)
	{
		httplib::Client cli(host.c_str(), port);
		auto res = cli.Get(fmt::format("/api/argument/{}?pt=true", id).c_str());
		if (!res)
		{
			throw std::runtime_error(fmt::format("Fetch configuration from url failed, got no response."));
		}
		else if (res->status != 200)
		{
			throw std::runtime_error(fmt::format("Fetch configuration from url failed: {}", res->status));
		}
		try
		{
			LoadConfig(res->body);
		}
		catch (ConfigParseError& err)
		{
			SPDLOG_ERROR("Parse configuration from {} failed: {}", host, err.what());
			std::abort();
		}
		SPDLOG_TRACE("Load configuration from {}", host);
	}

	const ConfigType* operator->() const { return Instance(); }

	bool Valid() const noexcept { return m_Valid; }

private:
	bool m_Valid;
	ConfigType m_Config;
};

template <typename ConfigType>
class ConfigServiceBuilder
{
	using ServiceType = ConfigLoader<ConfigType>;
	using BaseBuilder = BaseServiceBuilder<ServiceType>;
	ApplicationBuilder& m_AppBuilder;
	BaseBuilder& m_BaseBuilder;
public:
	ConfigServiceBuilder(ApplicationBuilder& appBuilder, BaseBuilder& config)
		: m_AppBuilder(appBuilder), m_BaseBuilder(config)
	{
	}

	auto LoadFromUrl(std::string host, std::string id, const unsigned int port = 80) -> ConfigServiceBuilder&
	{
		m_BaseBuilder.AddConfig([host, id, port](ServiceType& option)
		{
			option.LoadConfigFromUrl(host, id, port);
		});
		return *this;
	}

	auto LoadFromFile(std::string filename) -> ConfigServiceBuilder&
	{
		m_BaseBuilder.AddConfig([filename](ServiceType& option)
		{
			option.LoadConfigFromFile(filename);
		});
		return *this;
	}
};
} //Utils
} //ossian


#endif // OSSIAN_CORE_CONFIGURATION
