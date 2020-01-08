#include "nv/Configuration.hpp"


#include <simdjson/jsonparser.h>
#include <spdlog/spdlog.h>
#include "nv/Configuration.hpp"

#include <string>
#include <unordered_map>
#include <variant>
#include <memory>

namespace NautilusVision
{
namespace Utils
{

Configuration::Configuration(std::string configFilename)
	:m_ConfigFilename(configFilename)
{
	LoadConfig(configFilename);
}

bool Configuration::LoadConfig()
{
	m_Content.clear();

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

#ifdef _DEBUG
	for (auto&& item : m_Content)
	{
		spdlog::info("Load Configuration: {}", item.first);
	}
#endif
}

bool Configuration::LoadConfig(std::string configFilename)
{
	m_ConfigFilename = configFilename;
	return LoadConfig();
}

void Configuration::ConfigIterProc(simdjson::ParsedJson::Iterator& it, std::string prefix)
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
			m_Content.insert(std::pair<std::string, ContentType>(prefix, Boolean(true)));
		else if (it.is_false())
			m_Content.insert(std::pair<std::string, ContentType>(prefix, Boolean(false)));
		else if (it.is_string())
			m_Content.insert(std::pair<std::string, ContentType>(prefix, String(it.get_string())));
		else if (it.is_integer())
			m_Content.insert(std::pair<std::string, ContentType>(prefix, Integer(it.get_integer())));
		else if (it.is_double())
			m_Content.insert(std::pair<std::string, ContentType>(prefix, Double(it.get_double())));
	}
}

std::unique_ptr<Configuration> CreateConfiguration()
{
	return std::make_unique<Configuration>("NVConfig.json");
}

} // Utils

} // NautilusVision
