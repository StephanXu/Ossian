
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

namespace ossian
{

namespace Utils
{


/**
 * @class	Configuration
 *
 * @brief	配置文件解析
 *
 * @author	Xu Zihan
 * @date	2019/11/21
 */
class ConfigLoader
{
public:
    ConfigLoader()
    {
    }

    template<typename ConfigType>
    void LoadConfig(std::string text)
    {
        m_Config.reset(new ConfigType);
        google::protobuf::util::JsonStringToMessage(text, m_Config.get());
#ifdef _DEBUG
        {
            std::string str;
            google::protobuf::util::JsonOptions opt;
            opt.always_print_primitive_fields = true;
            opt.add_whitespace = true;
            google::protobuf::util::MessageToJsonString(*m_Config, &str, opt);
            spdlog::info("{}", str);
        }
#endif
        m_Valid = true;
    }

    template<typename ConfigType>
    ConfigType* Instance()
    {
        if (m_Valid)
            return google::protobuf::DynamicCastToGenerated<ConfigType>(m_Config.get());
        return nullptr;
    }

    template<typename ConfigType>
    void LoadConfigFromFile(std::string configFilename)
    {
        std::ifstream f(configFilename);
        std::string text{ std::istreambuf_iterator<char>{f},std::istreambuf_iterator<char>{} };
        spdlog::info("Load configuration from file: {}", configFilename);
        LoadConfig<ConfigType>(text);
    }

    template<typename ConfigType>
    void LoadConfigFromUrl(std::string host, int port, std::string path)
    {
        httplib::Client cli(host, port);
        auto res = cli.Get(path.c_str());
        if (!res || res->status != 200)
        {
            throw std::runtime_error(fmt::format("Can't get configuration from url: {}", res->status));
        }
        spdlog::info("Load configuration from {}", host);
        LoadConfig<ConfigType>(res->body);
    }

    bool Valid() const noexcept { return m_Valid; }
private:
    bool m_Valid;
    std::shared_ptr<google::protobuf::Message> m_Config;
};

/**
 * @fn	std::unique_ptr<Configuration> CreateConfiguration()
 *
 * @brief	Creates the configuration
 *
 * @author	Xu Zihan
 * @date	2019/11/21
 *
 * @returns	The new configuration.
 */
template<typename ConfigType>
std::unique_ptr<ConfigLoader> CreateConfigLoader()
{
    auto loader = std::make_unique<ConfigLoader>();
    loader->LoadConfigFromUrl<ConfigType>("mrxzh.com", 5000, "/config");
    return std::move(loader);
}

} //Utils

} //ossian



#endif // OSSIAN_CORE_CONFIGURATION