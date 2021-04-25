#include "VisionModeSwitcher.hpp"

VisionModeSwitcherForRemote::VisionModeSwitcherForRemote(
	Aimbot* aimbot,
	ossian::IOData<RemoteStatus>* remoteStatus)
	: m_Aimbot(*aimbot)
	  , m_RemoteStatus(*remoteStatus)
{
	m_RemoteStatus.AddOnChange(
		[this](const RemoteStatus& a, const RemoteStatus& b)
		{
			SwitchMode(a, b);
		});
}

auto VisionModeSwitcherForRemote::Process(unsigned char* data) -> void
{
	// [TODO]: Simply if-else-if to enter correct process.
	if (m_VisionMode == VisionMode::Aimbot)
	{
		m_Aimbot.Process(data);
	}
}


auto VisionModeSwitcherForRemote::Mode() const -> ::VisionMode { return m_VisionMode; }

auto VisionModeSwitcherForRemote::SwitchMode(const RemoteStatus& val, const RemoteStatus& lastVal) -> void
{
	// [TODO]: This is an example for switching! Just set m_VisionMode to correct mode.
	if (val.sw[1] == 1)
	{
		m_VisionMode = VisionMode::Aimbuff;
		return;
	}
	m_VisionMode = VisionMode::Aimbot;
}
