/**
 * @file	ossian\io\CAN.hpp
 *
 * @brief	Declares the can class
 */
#ifndef OSSIAN_CORE_IO_CAN
#define OSSIAN_CORE_IO_CAN
#ifdef __linux__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

#include <functional>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>
#include <cstring>
#include <tuple>
#include <exception>

#include "IO.hpp"
#include "IOError.hpp"

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

class CANBus : public IIOBus, public std::enable_shared_from_this<CANBus>
{
public:
	CANBus() = delete;
	CANBus(std::shared_ptr<CANManager> manager, std::string location, bool isLoopback);
	CANBus(const CANBus& canDevice) = delete;
	~CANBus();
	std::shared_ptr<IIOManager> Manager() override;
	FileDescriptor FD() const noexcept override;
	std::string Location() const noexcept override;
	bool IsOpened() const noexcept override;
	bool Open() override;
	bool Close() override;
	void Read() override;
	std::vector<std::shared_ptr<BaseDevice>> GetDevices() override;

	std::shared_ptr<BaseDevice> AddDevice(unsigned int id,
		std::function<ReceiveCallback> callback);
	void WriteRaw(unsigned int id, size_t length, std::shared_ptr<uint8_t[]> data);

private:
	bool m_IsOpened;
	bool m_IsLoopback;
	FileDescriptor m_FD;
	std::string m_Location;
	std::unordered_map<unsigned int, std::shared_ptr<CANDevice>> m_DeviceMap;
	std::shared_ptr<CANManager> m_Manager;
	void UpdateFilter();
};

class CANDevice : public BaseDevice, public std::enable_shared_from_this<CANDevice>
{
public:
	CANDevice() = delete;
	CANDevice(std::shared_ptr<CANBus> bus,
		unsigned int id,
		std::function<ReceiveCallback> callback) noexcept;

	std::shared_ptr<IIOBus> Bus() override;
	void Invoke(size_t length, std::shared_ptr<uint8_t[]> data) override;
	void WriteRaw(size_t length, std::shared_ptr<uint8_t[]> data) override;
	void SetCallback(std::function<ReceiveCallback> callback) override; //非线程安全

private:
	const unsigned int m_Id;
	std::shared_ptr<CANBus> m_Bus;
	std::function<ReceiveCallback> m_Callback;
};

class CANManager : public IIOManager, public std::enable_shared_from_this<CANManager>
{
public:
	CANManager() = default;

	IOType Type() const noexcept override;

	// 注册设备
	std::shared_ptr<IIOBus> AddBus(std::string location) override;
	std::shared_ptr<IIOBus> AddBus(std::string location, bool isLoopback);

	bool DelBus(std::shared_ptr<IIOBus> bus) override;
	bool DelBus(std::string location) override;

	void WriteTo(std::shared_ptr<BaseDevice> device, size_t length, std::shared_ptr<uint8_t[]> data) override;

	std::shared_ptr<BaseDevice> AddDevice(std::shared_ptr<CANBus> bus, unsigned int id,
		std::function<ReceiveCallback> callback);
	std::shared_ptr<BaseDevice> AddDevice(std::string location,
		const unsigned int id,
		const std::function<ReceiveCallback> callback);
	
	std::shared_ptr<IIOBus> Bus(std::string location) override;

	std::vector<std::shared_ptr<IIOBus>> GetBuses() override;

private:
	std::unordered_map<std::string, std::shared_ptr<IIOBus>> m_BusMap;
};

} // ossian

#endif // __linux__

#endif // OSSIAN_CORE_IO_CAN
