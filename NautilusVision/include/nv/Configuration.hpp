
#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <simdjson/jsonparser.h>

#include <string>
#include <unordered_map>
#include <variant>

namespace NautilusVision
{

namespace Utils
{
class Configuration
{
	using ContentType = std::variant<std::string, long long, double, bool>;
public:
	Configuration(std::string configFilename)
		:m_ConfigFilename(configFilename)
	{

	}

	bool LoadConfig()
	{
		simdjson::padded_string p = simdjson::get_corpus(m_ConfigFilename);
		simdjson::ParsedJson pj;
		pj.allocate_capacity(p.size());
		const int res = simdjson::json_parse(p, pj);
		if (0 != res)
		{
			spdlog::error("Parse configuration file failed: {}", simdjson::error_message(res));
			return false;
		}
		
		simdjson::ParsedJson::Iterator it(pj);
		ConfigIterProc(it, "");
	}

	bool LoadConfig(std::string configFilename)
	{
		m_ConfigFilename = configFilename;
		return LoadConfig();
	}

	ContentType& operator[](const std::string key)
	{
		return m_Content[key];
	}

private:

	void ConfigIterProc(simdjson::ParsedJson::Iterator& it, std::string prefix)
	{
		if (it.is_object())
		{
			if (it.down())
			{
				std::string nextPrefix{ it.get_string() };
				it.next();
				ConfigIterProc(it, prefix + "/" + nextPrefix);
				while (it.next())
				{
					nextPrefix = it.get_string();
					it.next();
					ConfigIterProc(it, prefix + "/" + nextPrefix);
				}
				it.up();
			}
		}
		else if (it.is_array())
		{
			return;
		}
		else
		{
			if (it.is_true())
				m_Content.insert(std::pair<std::string, ContentType>(prefix, true));
			else if (it.is_false())
				m_Content.insert(std::pair<std::string, ContentType>(prefix, false));
			else if (it.is_string())
				m_Content.insert(std::pair<std::string, ContentType>(prefix, it.get_string()));
			else if (it.is_integer())
				m_Content.insert(std::pair<std::string, ContentType>(prefix, it.get_integer()));
			else if (it.is_double())
				m_Content.insert(std::pair<std::string, ContentType>(prefix, it.get_double()));
		}
	}

	std::string m_ConfigFilename;

	std::unordered_map<std::string, ContentType> m_Content;
};
} // Utils

} // NautilusVision

#endif // CONFIGURATION_HPP