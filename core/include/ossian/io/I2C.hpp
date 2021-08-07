/**
 * @file	ossian\io\I2C.hpp
 *
 * @brief	Declares the i2c class
 */
#ifndef OSSIAN_CORE_IO_I2C
#define OSSIAN_CORE_IO_I2C

#include <spdlog/spdlog.h>

#ifdef __linux__
#include "IO.hpp"
#include "ossian/Factory.hpp"
namespace ossian
{

class I2CBus;
class I2CDevice;
class I2CManager;

class I2CBus
{
public:
	I2CBus() = delete;
	I2CBus(const I2CBus& other) = delete;
	explicit I2CBus(I2CManager* manager, std::string const& location)
		:m_IsOpened(false), m_Location(location), m_Manager(manager)
	{
		Open();
	}
	~I2CBus()
	{
		m_DeviceMap.clear();
		if(m_IsOpened) Close();
	};
	
	/*void ReadRaw(const size_t length, const uint8_t* data);
	void WriteRaw(const size_t length, const uint8_t* data);*/
	I2CManager* Manager() const noexcept { return m_Manager; }
	FileDescriptor FD() const noexcept { return m_FD; }
	const std::string Location() const noexcept { return m_Location; }
	bool IsOpened() const noexcept { return m_IsOpened; }
	bool Open();
	bool Close();
	std::shared_ptr<I2CDevice> GetOrAddDevice(unsigned address);
private:
	bool m_IsOpened;
	FileDescriptor m_FD;
	std::string m_Location;
	std::unordered_map<unsigned, std::shared_ptr<I2CDevice>> m_DeviceMap;
	I2CManager* m_Manager;
};

class I2CDevice : public std::enable_shared_from_this<I2CDevice>
{
public:
	I2CDevice() = delete;
	I2CDevice(const I2CDevice& other) = delete;
	explicit I2CDevice(I2CBus* bus, const unsigned int address) noexcept
		: m_Address(address), m_Bus(bus), m_IsOpened(false)
	{
		Open();
	};
	~I2CDevice()
	{
		Close();
	}
	bool IsOpened() const { return m_IsOpened; }
	bool Open();
	bool Close() { return true; } // [INFO] 可能需要额外操作 
	I2CBus* Bus() const { return m_Bus; }
	void ReadRaw(const size_t length, uint8_t* data);
	void WriteRaw(const size_t length, const uint8_t* data);

private:
	const unsigned int m_Address;
	I2CBus* m_Bus;
	bool m_IsOpened;
};

class I2CManager
{
public:
	OSSIAN_SERVICE_SETUP(I2CManager());
	I2CManager(const I2CManager& other) = delete;
	I2CBus* Bus(std::string const& location) const;
	std::shared_ptr<I2CDevice> GetOrAddDevice(std::string const& location, const unsigned address);

private:
	std::unordered_map<std::string, std::shared_ptr<I2CBus>> m_BusMap;
	I2CBus* AddBus(std::string const& location);
	bool DelBus(std::string const& location);
};

} // ossian

#endif // __linux__

#endif // OSSIAN_CORE_IO_I2C
