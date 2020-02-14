/**
 * @file	ossian\io\CAN.hpp
 *
 * @brief	Declares the can class
 */
#ifndef OSSIAN_CORE_IO_CAN
#define OSSIAN_CORE_IO_CAN
#ifdef __linux__

#include <functional>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>
#include "IO.hpp"

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
	explicit CANBus(std::shared_ptr<CANManager> manager, std::string location, bool isLoopback);
	CANBus(const CANBus&) = delete;
	~CANBus();
	const std::shared_ptr<IIOManager> Manager() override { return std::static_pointer_cast<IIOManager>(m_Manager); }
	FileDescriptor FD() const noexcept override { return m_FD; }
	const std::string Location() const noexcept override { return m_Location; }
	bool IsOpened() const noexcept override { return m_IsOpened; }
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
	explicit CANDevice(std::shared_ptr<CANBus> bus,
			  unsigned int id,
			  std::function<ReceiveCallback> callback) noexcept;
	CANDevice(const CANDevice&) = delete;

	const std::shared_ptr<IIOBus> Bus() const override { return std::dynamic_pointer_cast<IIOBus>(m_Bus); }
	void Invoke(const size_t length, std::shared_ptr<uint8_t[]> const& data) override { m_Callback(shared_from_this(), length, data); }
	void WriteRaw(const size_t length, std::shared_ptr<uint8_t[]> const& data) override { m_Bus->WriteRaw(m_Id, length, data); }
	void SetCallback(std::function<ReceiveCallback> const& callback) override { m_Callback = callback; }

private:
	const unsigned int m_Id;
	std::shared_ptr<CANBus> m_Bus;
	std::function<ReceiveCallback> m_Callback;
};

class CANManager : public IIOManager, public std::enable_shared_from_this<CANManager>
{
public:
	explicit CANManager() = default;
	CANManager(const CANManager&) = delete;
	IOType Type() const noexcept override { return IOType::CAN; }

	// 注册设备
	const std::shared_ptr<IIOBus> AddBus(std::string const& location) override;
	const std::shared_ptr<IIOBus> AddBus(std::string const& location, bool isLoopback);

	bool DelBus(std::shared_ptr<IIOBus> bus) override;
	bool DelBus(std::string const& location) override;

	void WriteTo(std::shared_ptr<BaseDevice> const& device, size_t length, std::shared_ptr<uint8_t[]> const& data) override;

	const std::shared_ptr<BaseDevice> AddDevice(std::shared_ptr<CANBus> const& bus, const unsigned id,
												std::function<ReceiveCallback> const& callback);
	const std::shared_ptr<BaseDevice> AddDevice(std::string const& location,
												const unsigned int id,
												const std::function<ReceiveCallback> callback);

	const std::shared_ptr<IIOBus> Bus(std::string const& location) const override;

	const std::vector<std::shared_ptr<IIOBus>> GetBuses() const override;

private:
	std::unordered_map<std::string, std::shared_ptr<IIOBus>> m_BusMap;
};

} // ossian

#endif // __linux__

#endif // OSSIAN_CORE_IO_CAN
