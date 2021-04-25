#ifndef OSSIAN_VISIONMODESWITCHER
#define OSSIAN_VISIONMODESWITCHER

#include <ossian/Factory.hpp>

#include "Aimbot.hpp"
#include "Remote.hpp"

enum class VisionMode
{
	Aimbot,
	Aimbuff
};

class IVisionModeSwitcher
{
public:
	virtual ~IVisionModeSwitcher() = default;
	virtual auto Process(unsigned char* data) -> void = 0;
	[[nodiscard]] virtual auto Mode() const -> VisionMode = 0;
};

class VisionModeSwitcherForRemote : public IVisionModeSwitcher
{
public:
	OSSIAN_SERVICE_SETUP(
		VisionModeSwitcherForRemote(
			Aimbot* aimbot,
			ossian::IOData<RemoteStatus>* remoteStatus)
		);

	~VisionModeSwitcherForRemote() = default;

	auto Process(unsigned char* data) -> void override;

	[[nodiscard]] auto Mode() const -> ::VisionMode override;

private:

	auto SwitchMode(const RemoteStatus& val, const RemoteStatus& lastVal) -> void;

	VisionMode m_VisionMode;
	Aimbot& m_Aimbot;
	ossian::IOData<RemoteStatus>& m_RemoteStatus;
};

#endif // OSSIAN_VISIONMODESWITCHER