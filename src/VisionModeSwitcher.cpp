#include "VisionModeSwitcher.hpp"

VisionModeSwitcherForRemote::VisionModeSwitcherForRemote(
	Aimbot* aimbot,
	Aimbuff* aimbuff
#ifndef VISION_ONLY
	, ossian::IOData<RemoteStatus>* remoteStatus
#endif // !VISION_ONLY
)
      : m_Aimbot(*aimbot)
	  , m_Aimbuff(*aimbuff)
#ifndef VISION_ONLY
	  , m_RemoteStatus(*remoteStatus)
#endif // !VISION_ONLY
{
#ifndef VISION_ONLY
	m_RemoteStatus.AddOnChange(
		[this](const RemoteStatus& a, const RemoteStatus& b)
		{
			SwitchMode(a, b);
		});
#endif // !VISION_ONLY
}

auto VisionModeSwitcherForRemote::Process(unsigned char* data) -> void
{
	// [TODO]: Simply if-else-if to enter correct process.
	if (m_VisionMode == VisionMode::Aimbot)
	{
		m_Aimbot.Process(data);
	}
	else if (m_VisionMode == VisionMode::Aimbuff)
	{
		m_Aimbuff.Process(data);
	}
}


auto VisionModeSwitcherForRemote::Mode() const -> ::VisionMode { return m_VisionMode; }

auto VisionModeSwitcherForRemote::SwitchMode(const RemoteStatus& val, const RemoteStatus& lastVal) -> void
{
	// [TODO]: This is an example for switching! Just set m_VisionMode to correct mode.
	/*if (val.sw[1] == 1)
	{
		m_VisionMode = VisionMode::Aimbuff;
		return;
	}*/
	m_VisionMode = VisionMode::Aimbot;
	//m_VisionMode = VisionMode::Aimbuff;
}
