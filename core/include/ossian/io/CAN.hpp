/**
 * @file	ossian\io\CAN.hpp
 *
 * @brief	Declares the can class
 */
#ifndef OSSIAN_CORE_IO_CAN
#define OSSIAN_CORE_IO_CAN

#include <spdlog/spdlog.h>

#ifdef __linux__
#include <termios.h>
#include <fcntl.h> 
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

#include <functional>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>

#include "IO.hpp"
#include "ossian/IOListener.hpp"
#include "ossian/Factory.hpp"
#include "ossian/MultiThread.hpp"
namespace ossian
{

/**
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
class CANBus;
class CANDevice;
class CANManager;

class CANBus : public IListenable
{
public:
	CANBus() = delete;
	CANBus(const CANBus& other) = delete;
	explicit CANBus(CANManager* manager, std::string const& location, bool isLoopback);
	~CANBus();

	CANManager* Manager() const noexcept { return m_Manager; }
	FileDescriptor FD() const noexcept { return m_FD; }
	const std::string Location() const noexcept { return m_Location; }
	bool IsOpened() const noexcept { return m_IsOpened; }
	bool Open();
	bool Close();
	void Read() override;
	const std::vector<CANDevice*> GetDevices();

	std::shared_ptr<CANDevice> AddDevice(unsigned int id);
	void WriteRaw(const unsigned id, const size_t length, const uint8_t* data) const;

private:
	bool m_IsOpened;
	bool m_IsLoopback;
	FileDescriptor m_FD;
	std::string m_Location;
	std::unordered_map<unsigned int, std::shared_ptr<CANDevice>> m_DeviceMap;
	CANManager* m_Manager;
	void UpdateFilter();
	ThreadPool m_ThreadPool;
};

class CANDevice : public BaseDevice, public std::enable_shared_from_this<CANDevice>
{
public:
	CANDevice() = delete;
	CANDevice(const CANDevice& other) = delete;
	explicit CANDevice(CANBus* bus, const unsigned int id) noexcept
	: m_Id(id), m_Bus(bus), m_Callback(DefaultCallback)
	{};

	CANBus* Bus() const { return m_Bus; }
	void Invoke(const size_t length, const uint8_t* data) override { m_Callback(shared_from_this(), length, data); }
	void WriteRaw(const size_t length, const uint8_t* data) const override { m_Bus->WriteRaw(m_Id, length, data); }

	std::shared_ptr<CANDevice> SetCallback(std::function<ReceiveCallback<CANDevice>> const& callback)
	{
		m_Callback = callback;
		return shared_from_this();
	}

private:
	const unsigned int m_Id;
	CANBus* m_Bus;
	std::function<ReceiveCallback<CANDevice>> m_Callback;
	static void DefaultCallback(std::shared_ptr<CANDevice> const&, const size_t, const uint8_t*)
	{}
};

class CANManager: private Attachable<1>
{
public:
	OSSIAN_SERVICE_SETUP(CANManager(IOListener* listener));
	CANManager(const CANManager& other) = delete;

	void WriteTo(const BaseDevice* device, const size_t length, const uint8_t* data);
	std::shared_ptr<CANDevice> AddDevice(std::string const& location, const unsigned int id);
	CANBus* Bus(std::string const& location) const;
	std::vector<CANBus*> GetBuses() const;

private:
	std::unordered_map<std::string, std::shared_ptr<CANBus>> m_BusMap;
	CANBus* AddBus(std::string const& location, bool isLoopback = false);
	bool DelBus(std::string const& location);
};

} // ossian

#endif // __linux__

#endif // OSSIAN_CORE_IO_CAN
