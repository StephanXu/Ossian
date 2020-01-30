#ifndef OSSIAN_CORE_IO_DEVICEMAP
#define OSSIAN_CORE_IO_DEVICEMAP
#ifdef __linux__
#include <vector>
#include <unordered_map>

#include "IO.hpp"
#include "IOTypes.hpp"

namespace ossian
{
	template <typename T>
	class DeviceMap
	{
	public:
		bool Insert(std::shared_ptr<T> dev)
		{
			auto location = dev->Location();
			auto fd = dev->FD();
			auto itLocation = m_LocationMap.find(location);
			auto itFD = m_FDMap.find(fd);
			if (itLocation == m_LocationMap.end() && itFD == m_FDMap.end())
			{
				m_LocationMap.insert(std::make_pair(location, dev));
				m_FDMap.insert(std::make_pair(fd, dev));
				return true;
			}
			return false;
		}
		bool Erase(std::string location)
		{
			auto itLocation = m_LocationMap.find(location);
			auto fd = itLocation->second->FD();
			auto itFD = m_FDMap.find(fd);
			if (itLocation != m_LocationMap.end() && itFD != m_FDMap.end())
			{
				m_LocationMap.erase(itLocation);
				m_FDMap.erase(itFD);
				return true;
			}
			return false;
		}
		bool Erase(FileDescriptor fd)
		{
			auto itFD = m_FDMap.find(fd);
			auto location = itFD->second->Location();
			auto itLocation = m_LocationMap.find(location);
			if (itLocation != m_LocationMap.end() && itFD != m_FDMap.end())
			{
				m_LocationMap.erase(itLocation);
				m_FDMap.erase(itFD);
				return true;
			}
			return false;
		}
		std::shared_ptr<T> Find(FileDescriptor fd)
		{
			auto it = m_FDMap.find(fd);
			if(it != m_FDMap.end())
			{
				return it->second;
			}
			return nullptr;
		}
		std::shared_ptr<T> Find(std::string location)
		{
			auto it = m_LocationMap.find(location);
			if (it != m_LocationMap.end())
			{
				return it->second;
			}
			return nullptr;
		}

		typedef typename std::unordered_map<FileDescriptor, std::shared_ptr<T>>::iterator iterator;
		typedef typename std::unordered_map<FileDescriptor, std::shared_ptr<T>>::const_iterator const_iterator;
		iterator begin() noexcept
		{
			return m_FDMap.begin();
		}
		iterator end() noexcept
		{
			return m_FDMap.end();
		}
		const_iterator cbegin() const noexcept
		{
			return m_FDMap.begin();
		}
		const_iterator cend() const noexcept
		{
			return m_FDMap.end();
		}
		
	private:
		std::unordered_map<FileDescriptor, std::shared_ptr<T>> m_FDMap;
		std::unordered_map<std::string, std::shared_ptr<T>> m_LocationMap;
	};
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO_DEVICEMAP