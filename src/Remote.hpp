#ifndef OSSIAN_REMOTE_HPP
#define OSSIAN_REMOTE_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/MultiThread.hpp>
#include <ossian/IOData.hpp>

#include <mutex>

struct RemoteStatus
{
	/* rocker channel information */
	int16_t ch[5];

	/* left and right lever information */
	uint8_t sw[2];
};

class IRemote
{
public:
	virtual ~IRemote() = default;
	virtual auto AddRemote(std::string location) -> void = 0;
};

template <typename Mutex = std::mutex>
class Remote final : public IRemote, public ossian::IODataBuilder<Mutex, RemoteStatus>
{
	ossian::IOData<RemoteStatus>* m_IOData;
	ossian::UARTManager* m_UARTManager;
	RemoteStatus m_Status;
	Mutex m_Mutex;
public:
	OSSIAN_SERVICE_SETUP(Remote(ossian::UARTManager* uartManager,
		ossian::IOData<RemoteStatus>* ioData))
		: m_UARTManager(uartManager), m_IOData(ioData)
	{
	}

	virtual ~Remote() = default;

	auto AddRemote(std::string location) -> void override
	{
		using namespace ossian::UARTProperties;
		m_UARTManager->AddDevice(location,
		                         R115200,
		                         FlowControlNone,
		                         DataBits8,
		                         StopBits1,
		                         ParityNone)
		             ->SetCallback(
			             [this](const std::shared_ptr<ossian::BaseDevice>& device, const size_t length,
			                    const uint8_t* data)
			             {
							 spdlog::trace("Remote Receive: {}", length);
				             sscanf(reinterpret_cast<const char*>(data),
				                    "CH1:%hu,CH2:%hu,CH3:%hu,CH4:%hu,CH5:%hu,S1:%hhu,S2:%hhu",
				                    &m_Status.ch[0],
				                    &m_Status.ch[1],
				                    &m_Status.ch[2],
				                    &m_Status.ch[3],
				                    &m_Status.ch[4],
				                    &m_Status.sw[0],
				                    &m_Status.sw[1]);
				             m_IOData->Set(m_Status);
			             });
	}
};

using RemoteMt = Remote<std::mutex>;
using RemoteSt = Remote<ossian::null_mutex>;

#endif // OSSIAN_REMOTE_HPP
