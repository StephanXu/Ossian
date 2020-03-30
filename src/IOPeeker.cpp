#include "IOPeeker.hpp"

auto IOPeeker::ExecuteProc() -> void
{
	auto logger = spdlog::get("OnlineLog");
	while (true)
	{
		logger->trace("Listener,{},{}",logger->name(), spdlog::default_logger()->name());
		m_Listener->Listen(1000);
	}
}
