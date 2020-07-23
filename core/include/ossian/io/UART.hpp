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
class UARTInitializeFailed : public std::runtime_error
{
public:
	UARTInitializeFailed(const std::string message) : std::runtime_error(message)
	{}
};

namespace UARTProperties
{
const std::unordered_map<int, unsigned int> Baudrate = 
{
	{0,0000000},
	{50, 0000001},
	{75, 0000002},
	{110, 0000003},
	{134, 0000004},
	{150, 0000005},
	{200, 0000006},
	{300, 0000007},
	{600, 0000010},
	{1200, 0000011},
	{1800, 0000012},
	{2400, 0000013},
	{4800, 0000014},
	{9600, 0000015},
	{19200, 0000016},
	{38400, 0000017},
	{57600, 0010001},
	{115200, 0010002},
	{230400, 0010003},
	{460800, 0010004},
	{500000, 0010005},
	{576000, 0010006},
	{921600, 0010007},
	{1000000, 0010010},
	{1152000, 0010011},
	{1500000, 0010012},
	{2000000, 0010013},
	{2500000, 0010014},
	{3000000, 0010015},
	{3500000, 0010016},
	{4000000, 0010017}
};
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
	UARTDevice* GetDevice() const { return m_Device.get(); }
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
		if(length)
		{
			m_Callback(shared_from_this(), length, data.get());
		}
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
	void ProcessDevices();

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
