/**
 * @file	ossian\io\UART.hpp
 *
 * @brief	Declares the UART class
 */
#ifndef OSSIAN_CORE_IO_UART
#define OSSIAN_CORE_IO_UART

#ifdef __linux__

#include <functional>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>
#include "IO.hpp"
#include "ossian/Factory.hpp"
#include "ossian/IOListener.hpp"
#include "ossian/io/BufferPool.hpp"

namespace ossian
{
namespace UARTProperties
{

enum DataBits
{
	DataBits5 = 0000000,
	DataBits6 = 0000020,
	DataBits7 = 0000040,
	DataBits8 = 0000060
};

enum StopBits
{
	StopBits1 = 0,
	StopBits2 = 1,
};

enum Parity
{
	ParityNone = 0,
	ParityEven = 1,
	ParityOdd = 2
};

enum FlowControl
{
	FlowControlNone = 0,
	FlowControlHardware = 1,
	FlowControlSoftware = 2
};
} // UARTProperties

class UARTBus;
class UARTDevice;
class UARTManager;

class UARTBus : public IListenable
{
public:
	UARTBus() = delete;
	explicit UARTBus(UARTManager* manager,
	                 std::string const& location,
	                 const unsigned long baudrate,
	                 const UARTProperties::FlowControl flowctrl,
	                 const UARTProperties::DataBits databits,
	                 const UARTProperties::StopBits stopbits,
	                 const UARTProperties::Parity parity);
	UARTBus(const UARTBus& other) = delete;
	~UARTBus();
	UARTManager* Manager() const { return m_Manager; }
	FileDescriptor FD() const noexcept override { return m_FD; }
	std::string Location() const noexcept { return m_Location; }
	bool IsOpened() const noexcept { return m_IsOpened; }
	bool Open();
	bool Close();
	void Read() const override;
	std::vector<UARTDevice*> GetDevices() const;

	std::shared_ptr<UARTDevice> const& AddDevice();
	void WriteRaw(size_t length, const uint8_t* data) const;

private:
	bool m_IsOpened;
	FileDescriptor m_FD;
	std::string m_Location;

	unsigned long m_Baudrate;
	UARTProperties::FlowControl m_FlowCtrl;
	UARTProperties::DataBits m_DataBits;
	UARTProperties::StopBits m_StopBits;
	UARTProperties::Parity m_Parity;

	std::shared_ptr<UARTDevice> m_Device;
	UARTManager* m_Manager;
};

class UARTDevice : public BaseDevice, public std::enable_shared_from_this<UARTDevice>
{
public:
	UARTDevice() = delete;

	explicit UARTDevice(UARTBus* bus) noexcept : m_Bus(bus), m_Callback(DefaultCallback)
	{
		m_BufferPool = std::make_unique<BufferPool>(2);
	};
	UARTDevice(const UARTDevice& other) = delete;

	UARTBus* Bus() const { return m_Bus; }

	void Invoke(const size_t length, std::shared_ptr<uint8_t[]> data) override
	{
		m_BufferPool->AddTask(
			[callback = m_Callback, ptr = shared_from_this(), length, data = std::move(data)]()
			{
				callback(ptr, length, data.get());
			});
	}

	void Process() const { return m_BufferPool->Process(); }
	void WriteRaw(const size_t length, const uint8_t* data) const override { m_Bus->WriteRaw(length, data); }

	std::shared_ptr<BaseDevice> SetCallback(std::function<ReceiveCallback<UARTDevice>> const& callback)
	{
		m_Callback = callback;
		return shared_from_this();
	}

private:
	UARTBus* m_Bus;
	std::function<ReceiveCallback<UARTDevice>> m_Callback;
	std::unique_ptr<BufferPool> m_BufferPool;

	static void DefaultCallback(std::shared_ptr<UARTDevice> const&, const size_t, const uint8_t*)
	{
	}
};

class UARTManager : private Attachable<0>
{
public:
	OSSIAN_SERVICE_SETUP(UARTManager(IOListener* listener));
	UARTManager(const UARTManager& other) = delete;

	void WriteTo(const UARTDevice* device, const size_t length, const uint8_t* data);
	std::shared_ptr<UARTDevice> AddDevice(std::string const& location,
	                                      const unsigned long baudrate,
	                                      const UARTProperties::FlowControl flowctrl,
	                                      const UARTProperties::DataBits databits,
	                                      const UARTProperties::StopBits stopbits,
	                                      const UARTProperties::Parity parit);
	UARTBus* Bus(std::string const& location) const;
	std::vector<UARTBus*> GetBuses() const;
	std::vector<UARTDevice*> GetDevices() const;

private:
	std::unordered_map<std::string, std::shared_ptr<UARTBus>> m_BusMap;

	UARTBus* AddBus(std::string const& location,
	                const unsigned long baudrate,
	                const UARTProperties::FlowControl flowctrl,
	                const UARTProperties::DataBits databits,
	                const UARTProperties::StopBits stopbits,
	                const UARTProperties::Parity parity);

	bool DelBus(std::string const& location);
};

const size_t MAX_LENGTH = 8192;
} // ossian

#endif // __linux__

#endif // OSSIAN_CORE_IO_UART
