
#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <simdjson/jsonparser.h>
#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <string>
#include <unordered_map>
#include <variant>
#include <memory>

namespace NautilusVision
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
class Configuration
{
	using ContentType = std::variant<std::string, long long, double, bool>;
public:

	using String = std::string;
	using Integer = long long;
	using Double = double;
	using Boolean = bool;

	class ConfigLoadError : public std::runtime_error
	{
	public:
		ConfigLoadError(std::string path)
			:std::runtime_error(fmt::format("Load configuration {} failed: key doesn't exist.", path))
			, m_Path(path)
		{
		}

		std::string Path() { return m_Path; }
	private:
		std::string m_Path;
	};

	/**
	 * @fn	Configuration::Configuration(std::string configFilename)
	 *
	 * @brief	Constructor
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/21
	 *
	 * @param	configFilename	Filename of the configuration file.
	 */
	Configuration(std::string configFilename);

	/**
	 * @fn	bool Configuration::LoadConfig()
	 *
	 * @brief	Loads the configuration from file
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/21
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	bool LoadConfig();

	/**
	 * @fn	bool Configuration::LoadConfig(std::string configFilename)
	 *
	 * @brief	Loads a configuration from file
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/21
	 *
	 * @param	configFilename	Filename of the configuration file.
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	bool LoadConfig(std::string configFilename);

	/**
	 * @fn	template<typename T> T Configuration::LoadValue(std::string path)
	 *
	 * @brief	将配置存储到变量
	 * 请尽可能使用两个参数的LoadValue函数以控制异常，若执意使用此函数请自行处理异常
	 * @exception	ConfigLoadError	Raised when a Configuration Load error condition occurs.
	 *
	 * @tparam	T	配置数据类型
	 * @param	path	字段路径
	 *
	 * @returns	The value.
	 */
	template<typename T>
	T LoadValue(std::string path) const
	{
		try
		{
			auto it = m_Content.find(path);
			if (it == m_Content.end())
			{
				throw ConfigLoadError(path);
			}
			return std::get<T>(it->second);
		}
		catch (std::bad_variant_access & e)
		{
			spdlog::error(e.what());
			throw ConfigLoadError(path);
		}
	}

	/**
	 * @fn	template<typename T> void Configuration::LoadValue(T& dest,std::string path)
	 *
	 * @brief	将配置存储到变量
	 *
	 * @tparam	T	配置数据类型
	 * @param [out]		dest	目标变量
	 * @param 		  	path	字段路径
	 */
	template<typename T>
	void LoadValue(T& dest, std::string path) const noexcept
	{
		T tmp;
		try
		{
			tmp = std::move(LoadValue<T>(path));
		}
		catch(ConfigLoadError& e)
		{
			spdlog::error(e.what());
		}
		dest = std::move(tmp);
	}

	void LoadStringValue(String& dest, std::string path) const noexcept { LoadValue<std::string>(dest, path); }
	String LoadStringValue(std::string path) const { return LoadValue<std::string>(path); }

	void LoadIntegerValue(Integer& dest, std::string path) const noexcept { LoadValue<Integer>(dest, path); }
	Integer LoadIntegerValue(std::string path) const { return LoadValue<Integer>(path); }

	void LoadDoubleValue(Double& dest, std::string path) const noexcept { LoadValue<Double>(dest, path); }
	Double LoadDoubleValue(std::string path) const { return LoadValue<Double>(path); }

	void LoadBooleanValue(Boolean& dest, std::string path) const noexcept { LoadValue<Boolean>(dest, path); }
	Boolean LoadBooleanValue(std::string path) const { return LoadValue<Boolean>(path); }
private:

	/**
	 * @fn	void Configuration::ConfigIterProc(simdjson::ParsedJson::Iterator& it, std::string prefix)
	 *
	 * @brief	Configuration iterator procedure
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/21
	 *
	 * @param [in]		it	  	The iterator.
	 * @param 		  	prefix	The prefix of key path.
	 */
	void ConfigIterProc(simdjson::ParsedJson::Iterator& it, std::string prefix);

	std::string m_ConfigFilename;

	std::unordered_map<std::string, ContentType> m_Content;
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
std::unique_ptr<Configuration> CreateConfiguration();

} // Utils

} // NautilusVision

#endif // CONFIGURATION_HPP