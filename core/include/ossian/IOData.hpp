#ifndef OSSIAN_CORE_IODATA
#define OSSIAN_CORE_IODATA

#include "Factory.hpp"

#include <mutex>
#include <typeindex>

namespace ossian
{
class IIOData
{
public:
	virtual ~IIOData() = default;
	virtual auto Lock() -> void = 0;
	virtual auto UnLock() -> void = 0;
	virtual auto TryLock() noexcept -> bool = 0;
	virtual auto TypeIndex() noexcept -> std::type_index = 0;
};

template <typename DataType, typename Mutex = std::mutex>
class IOData : public IIOData
{
public:
	using Type = DataType;
	using OnReceiveProcType = void(const DataType& value);

	OSSIAN_SERVICE_SETUP(IOData()) = default;
	~IOData()                      = default;
	IOData(const IOData&)          = delete;

	IOData(IOData&& listener) noexcept
	{
		*this = std::move(listener);
	}

	auto operator=(const IOData&) -> IOData& = delete;

	auto operator=(IOData&& listener) noexcept -> IOData&
	{
		{
			std::lock_guard<Mutex> guard{m_Mutex};
			m_Payload = std::move(listener.m_Payload);
		}
		m_Mutex    = std::move(listener.m_Mutex);
		m_OnChange = std::move(m_OnChange);
		return *this;
	}

	auto Set(const DataType& value) noexcept -> void
	{
		{
			std::lock_guard<Mutex>{m_Mutex};
			m_Payload = value;
		}
		m_OnChange(value);
	}

	auto Get() noexcept -> DataType
	{
		std::lock_guard<Mutex>{m_Mutex};
		return m_Payload;
	}

	auto GetRef() const noexcept -> const DataType&
	{
		return m_Payload;
	}

	auto Lock() -> void override
	{
		m_Mutex.lock();
	}

	auto UnLock() -> void override
	{
		m_Mutex.unlock();
	}

	auto TryLock() noexcept -> bool override
	{
		return m_Mutex.try_lock();
	}

	auto TypeIndex() noexcept -> std::type_index override
	{
		return std::type_index{typeid(DataType)};
	}

	auto AddOnChange(std::function<OnReceiveProcType> callback) -> void
	{
		m_OnChange = [callback, onChange = m_OnChange](const DataType& value)
		{
			onChange(value);
			callback(value);
		};
	}

private:
	DataType m_Payload;
	Mutex m_Mutex;
	std::function<OnReceiveProcType> m_OnChange = [](const DataType& value)
	{
	};
};
} // ossian

#endif // OSSIAN_CORE_IODATA
