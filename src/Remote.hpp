/**
 * @file Remote.hpp
 * @author Xu Zihan (mrxzh@outlook.com)
 * @brief Remote controller I/O logic
 * @version 0.1
 * @date 2020-02-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_REMOTE_HPP
#define OSSIAN_REMOTE_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/MultiThread.hpp>
#include <ossian/IOData.hpp>

#include <mutex>

/**
 * @brief Remote controller I/O data model
 */
struct RemoteStatus
{
	/* rocker channel information */
	int16_t ch[5];

	/* left and right lever information */
	uint8_t sw[2];
};

/**
 * @brief Remote controller service interface
 */
class IRemote
{
public:
	virtual ~IRemote() = default;

	/**
	 * @brief Initialize remote controller.
	 * 
	 * @param location The location of remote controller. (e.g. /dev/ttyTHS2)
	 */
	virtual auto AddRemote(std::string location) -> void = 0;
};

/**
 * @brief Remote IO service
 * 
 * @tparam std::mutex Mutex type
 */
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
							 spdlog::trace("Remote Receive: {}, buffer: {}", length, data);
							 while ((readPtr - data >= 0) && ('C' != readPtr[0] || 'H' != readPtr[1] || '1' != readPtr[2]))
							 {
								 if (length == 54)spdlog::error("Remote: Maybe overflow happened");
								 --readPtr;
							 }
							 if (readPtr < data)
							 {
								 spdlog::warn("Remote: Incomplete data.");
								 return;
							 }
				             
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
