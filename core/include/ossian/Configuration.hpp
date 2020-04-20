#ifndef OSSIAN_CORE_CONFIGURATION
#define OSSIAN_CORE_CONFIGURATION

#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/json_util.h>

#include <string>
#include <unordered_map>
#include <variant>
#include <memory>

#include "Http.hpp"
#include "ApplicationBuilder.hpp"

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
 * @author	Xu Zihan
 * @date	2019/11/21
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
		m_Config.reset(new ConfigType);
		const auto convertStatus = google::protobuf::util::JsonStringToMessage(text, m_Config.get());
		if (!convertStatus.ok())
		{
			throw ConfigParseError{convertStatus.ToString()};
		}
#ifdef _DEBUG
		{
			std::string str;
			google::protobuf::util::JsonOptions opt;
			opt.always_print_primitive_fields = true;
			opt.add_whitespace                = true;
			google::protobuf::util::MessageToJsonString(*m_Config, &str, opt);
			spdlog::info("{}", str);
		}
#endif
		m_Valid = true;
	}

	template <typename ConfigType>
	ConfigType* Instance()
	{
		if (m_Valid)
			return google::protobuf::DynamicCastToGenerated<ConfigType>(m_Config.get());
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
			spdlog::error("Parse configuration from file {} failed: {}", configFilename, err.what());
			std::abort();
		}
		spdlog::trace("Load configuration from file: {}", configFilename);
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
			spdlog::error("Parse configuration from {} failed: {}", host, err.what());
			std::abort();
		}
		spdlog::trace("Load configuration from {}", host);
	}

	bool Valid() const noexcept
	{
		return m_Valid;
	}

private:
	bool m_Valid;
	std::shared_ptr<google::protobuf::Message> m_Config;
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
