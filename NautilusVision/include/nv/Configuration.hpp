
#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <simdjson/jsonparser.h>

#include <string>

namespace NautilusVision
{

namespace Utils
{
class Configuration
{
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
		
		//simdjson::ParsedJson::Iterator it(pj);
	}

	bool LoadConfig(std::string configFilename)
	{
		m_ConfigFilename = configFilename;
		return LoadConfig();
	}

private:
	std::string m_ConfigFilename;
};
} // Utils

} // NautilusVision

#endif // CONFIGURATION_HPP