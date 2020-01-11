
#ifndef NAUTILUS_VISION_DEVICE_IO
#define NAUTILUS_VISION_DEVICE_IO

#include <nv/Configuration.hpp>

#include <functional>
#include <unordered_map>
#include <memory>
#include <vector>

namespace NautilusVision
{

class CanBus
{
public:
	CanBus() = delete;
	CanBus(unsigned int location, unsigned id) noexcept :m_Location(location) {}
	CanBus(const CanBus& canDevice) = delete;
	CanBus(CanBus&& canBus)
	{
		*this = std::move(canBus);
	}
	CanBus& operator=(CanBus&& canBus)
	{
		m_Location = canBus.m_Location;
	}

	~CanBus()
	{
		try
		{
			Close();
		}
		catch (std::exception & err)
		{
			std::abort();
		}
	}

	bool Open()
	{
		//[TODO]
	}

	void Close()
	{
		//[TODO]
	}

	unsigned int Location() const noexcept { return m_Location; }

	using ReceiveCallback = void(unsigned int id,
								 std::unique_ptr<uint8_t[]> data,
								 size_t dataLength);
	bool AddReceiveCallback(unsigned int id, std::function<ReceiveCallback> callback)
	{
		auto it = m_InIdMap.find(id);
		if (m_InIdMap.end() != it)
		{
			return false; //Maybe need exception
		}
		m_InIdMap.insert(std::make_pair(id, callback));
		return true;
	}

	bool RemoveReceiveCallback(unsigned int id)
	{
		auto it = m_InIdMap.find(id);
		if (m_InIdMap.end() == it)
		{
			return false;
		}
		m_InIdMap.erase(it);
		return true;
	}

	void ReadCan()
	{
		//[TODO]
		unsigned int id{};//parsed id.
		std::unique_ptr<uint8_t[]> buffer{};//raw data buffer
		size_t bufferSize{};//raw data buffer length

		auto it = m_InIdMap.find(id);
		if (m_InIdMap.end() == it)
		{
			return;
		}
		it->second(id, std::move(buffer), bufferSize);
	}

protected:
	unsigned int m_Location;
	std::unordered_map<unsigned int, std::function<ReceiveCallback>> m_InIdMap;
};

class CanManager
{
public:
	CanManager(Utils::Configuration* config)
	{
	}
private:
	std::vector<std::unique_ptr<CanBus>> m_Devices;
};

} //NautilusVision

#endif //NAUTILUS_VISION_DEVICE_IO