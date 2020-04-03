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
							 const size_t packSize = 54;
							 const uint8_t* readPtr = { data + length - packSize };
							 while (readPtr - data >= 0 && ('C' != readPtr[0] || 'H' != readPtr[1] || '1' != readPtr[2]))
							 {
								 --readPtr;
							 }
							 if (readPtr < data)
							 {
								 spdlog::warn("Remote: Incomplete data.");
								 return;
							 }
				             spdlog::trace("Remote Receive: {}, buffer: {}", length, data);
				             sscanf(reinterpret_cast<const char*>(readPtr),
				                    "CH1:%d,CH2:%d,CH3:%d,CH4:%d,CH5:%d,S1:%d,S2:%d",
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
