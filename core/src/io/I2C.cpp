#include "ossian/io/I2C.hpp"
#include <string>

#include <linux/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace ossian
{
I2CManager::I2CManager() = default;
// I2CBus
bool I2CBus::Open()
{	
	if(m_IsOpened)
	{
		spdlog::trace("I2C Bus {} is already opened.", m_Location);
		return false;
	}
	spdlog::trace("Opening i2c bus at {}", m_Location);
	m_FD = open(m_Location.c_str(), O_RDWR);
	if (m_FD < 0)
	{
		spdlog::warn("Failed to open {}, errno {}", m_Location, errno);
		return false;
	}
	m_IsOpened = true;
	return true;
}

bool I2CBus::Close()
{
	if (!m_IsOpened)
	{
		spdlog::trace("I2C Bus {} is already closed.", m_Location);
		return false;
	}
	close(m_FD);
	m_FD = -1;
	spdlog::trace("Closing i2c bus at {}", m_Location);
	m_IsOpened = false;
	return true;
}

std::shared_ptr<I2CDevice> I2CBus::GetOrAddDevice(unsigned address)
{
	const auto it = m_DeviceMap.find(address);
	std::shared_ptr<I2CDevice> device;
	if (it == m_DeviceMap.end())
	{
		device = std::make_shared<I2CDevice>(this, address);
		m_DeviceMap.insert(std::make_pair(address, device));
	}
	else
	{
		device = it->second;
	}
	return device;
}

bool I2CDevice::Open()
{
	if (m_IsOpened)
	{
		spdlog::trace("I2C Device {} is already opened. No need to call again.", m_Address);
		return false;
	}
	if (ioctl(m_Bus->FD(), I2C_SLAVE, m_Address) < 0)
	{
		spdlog::warn("I2C Device {} on {} failed to open, errno {}", m_Address, m_Bus->Location(), errno);
		return false;
	}
	m_IsOpened = true;
	return true;
}

void I2CDevice::ReadRaw(const size_t length, uint8_t* data)
{
	if(read(m_Bus->FD(), data, length) < 0)
	{
		spdlog::warn("I2C Device {} on {} failed to read, errno {}", m_Address, m_Bus->Location(), errno);
	}
}

void I2CDevice::WriteRaw(const size_t length, const uint8_t* data)
{
	if(write(m_Bus->FD(), data, length) < 0)
	{
		spdlog::warn("I2C Device {} on {} failed to write, errno {}", m_Address, m_Bus->Location(), errno);

	}
}

I2CBus* I2CManager::Bus(std::string const& location) const
{
	const auto it = m_BusMap.find(location);
	if (it == m_BusMap.end())
	{
		return nullptr;
	}
	return it->second.get();
}

std::shared_ptr<I2CDevice> I2CManager::GetOrAddDevice(std::string const& location, const unsigned address)
{
	auto bus = Bus(location);
	if (nullptr == bus)
	{
		bus = AddBus(location);
	}
	return bus->GetOrAddDevice(address);
}

I2CBus* I2CManager::AddBus(std::string const& location)
{
	auto bus = std::make_shared<I2CBus>(this, location);
	m_BusMap.insert(std::make_pair(location, bus));
	return bus.get();
}

bool I2CManager::DelBus(std::string const& location)
{
	auto bus = Bus(location);
	if (bus == nullptr)
	{
		return false;
	}
	return m_BusMap.erase(location);
}
}
