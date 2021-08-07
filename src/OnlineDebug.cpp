
#include "OnlineDebug.hpp"

OnlineDebugHub::OnlineDebugHub(signalr::hub_connection&& connection)
	: BaseHub(std::move(connection))
{
	HUB_REGISTER_CALLBACK(ReloadSettings);
	Start();
}

auto OnlineDebugHub::ReloadSettings(std::string settings) -> void
{
	//std::cout << settings << std::endl;
}
