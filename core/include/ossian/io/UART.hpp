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
namespace ossian
{

namespace UARTProperties
{
enum Baudrate
{
	R0 = 0000000,
	R50 = 0000001,
	R75 = 0000002,
	R110 = 0000003,
	R134 = 0000004,
	R150 = 0000005,
	R200 = 0000006,
	R300 = 0000007,
	R600 = 0000010,
	R1200 = 0000011,
	R1800 = 0000012,
	R2400 = 0000013,
	R4800 = 0000014,
	R9600 = 0000015,
	R19200 = 0000016,
	R38400 = 0000017,
	R57600 = 0010001,
	R115200 = 0010002,
	R230400 = 0010003,
	R460800 = 0010004,
	R500000 = 0010005,
	R576000 = 0010006,
	R921600 = 0010007,
	R1000000 = 0010010,
	R1152000 = 0010011,
	R1500000 = 0010012,
	R2000000 = 0010013,
	R2500000 = 0010014,
	R3000000 = 0010015,
	R3500000 = 0010016,
	R4000000 = 0010017
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
}// UARTProperties

class UARTBus;
class UARTDevice;
class UARTManager;

class UARTBus : public IListenable
{
public:
	UARTBus() = delete;
	explicit UARTBus(UARTManager* manager,
					 std::string const& location,
					 const UARTProperties::Baudrate baudrate,
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
	void Read() const;
	std::vector<UARTDevice*> GetDevices() const;

	UARTDevice* AddDevice();
	void WriteRaw(size_t length, const uint8_t* data) const;

private:
	bool m_IsOpened;
	FileDescriptor m_FD;
	std::string m_Location;

	UARTProperties::Baudrate m_Baudrate;
	UARTProperties::FlowControl m_FlowCtrl;
	UARTProperties::DataBits m_DataBits;
	UARTProperties::StopBits m_StopBits;
	UARTProperties::Parity m_Parity;

	std::shared_ptr<UARTDevice> m_Device;
	UARTManager* m_Manager;
};

class UARTDevice : public BaseDevice
{
public:
	UARTDevice() = delete;
	explicit UARTDevice(UARTBus* bus) noexcept;
	UARTDevice(const UARTDevice& other) = delete;

	UARTBus* Bus() const { return m_Bus; }
	void Invoke(const size_t length, const uint8_t* data) const override { m_Callback(this, length, data); }
	void WriteRaw(const size_t length, const uint8_t* data) const override { m_Bus->WriteRaw(length, data); }
	BaseDevice* SetCallback(std::function<ReceiveCallback> const& callback) override
	{
		m_Callback = callback;
		return this;
	}

private:
	UARTBus* m_Bus;
	std::function<ReceiveCallback> m_Callback;
};

class UARTManager
{
public:
	OSSIAN_SERVICE_SETUP(UARTManager()) = default;
	UARTManager(const UARTManager& other) = delete;

	void WriteTo(const UARTDevice* device, const size_t length, const uint8_t* data);
	UARTDevice* AddDevice(std::string const& location);
	UARTBus* Bus(std::string const& location) const;
	std::vector<UARTBus*> GetBuses() const;

private:
	std::unordered_map<std::string, std::shared_ptr<UARTBus>> m_BusMap;

	UARTBus* AddBus(std::string const& location);
	UARTBus* AddBus(std::string const& location,
	                const UARTProperties::Baudrate baudrate,
	                const UARTProperties::FlowControl flowctrl,
	                const UARTProperties::DataBits databits,
	                const UARTProperties::StopBits stopbits,
	                const UARTProperties::Parity parity);

	bool DelBus(std::string const& location);
};
const size_t MAX_LENGTH = 2048;
} // ossian

#endif // __linux__

#endif // OSSIAN_CORE_IO_UART
