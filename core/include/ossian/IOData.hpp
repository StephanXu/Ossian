/**
 * @file IOData.hpp
 * @author Xu Zihan (mrxzh@outlook.com)
 * @brief A basic data listener.
 * @version 0.1
 * @date 2020-03-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_CORE_IODATA
#define OSSIAN_CORE_IODATA

#include "Factory.hpp"
#include "ApplicationBuilder.hpp"

#include <mutex>
#include <typeindex>

namespace ossian
{
/**
 * @brief Interface of IOData.
 * 
 * @tparam DataType The model of payload.
 */
template <typename DataType>
class IOData
{
	using OnReceiveProcType = void(const DataType& value);
public:
	virtual ~IOData() = default;

	/**
	 * @brief Set value to payload.
	 * 
	 * @param value The value to set.
	 */
	virtual auto Set(const DataType& value) noexcept -> void = 0;

	/**
	 * @brief Get value of payload (Thread safe).
	 * 
	 * @return DataType The value of payload.
	 */
	virtual auto Get() noexcept -> DataType = 0;

	/**
	 * @brief Get the reference of payload (NOT thread safe).
	 * 
	 * @return const DataType& The reference of payload.
	 */
	virtual auto GetRef() const noexcept -> const DataType& = 0;

	/**
	 * @brief Wait until the next time the payload is set.
	 * 
	 * @return DataType The value of payload.
	 */
	virtual auto WaitNextValue() -> DataType = 0;

	/**
	 * @brief Locks the payload manually, blocks if the payload is not available.
	 */
	virtual auto Lock() -> void = 0;

	/**
	 * @brief Unlocks the payload manually.
	 */
	virtual auto UnLock() -> void = 0;

	/**
	 * @brief Lock the payload if not locked, returns if the payload is not available.
	 * 
	 * @return true The lock was acquired successfully.
	 * @return false The payload is not available currently.
	 */
	virtual auto TryLock() noexcept -> bool = 0;

	/**
	 * @brief Return the type index of payload.
	 * 
	 * @return std::type_index The type index of payload.
	 */
	virtual auto TypeIndex() noexcept -> std::type_index = 0;

	/**
	 * @brief Set a callback that be called when the payload is set.
	 * 
	 * @param callback The callback process.
	 */
	virtual auto AddOnChange(std::function<OnReceiveProcType> callback) -> void = 0;
};

template <typename T>
struct ConditionVariable
{
	using Type = std::condition_variable_any;
};

template <>
struct ConditionVariable<std::mutex>
{
	using Type = std::condition_variable;
};

/**
 * @brief An implementation of IOData with thread safe with basic function.
 * 
 * @tparam DataType The type of payload.
 * @tparam std::mutex 
 */
template <typename DataType, typename Mutex = std::mutex>
class IODataImpl : public IOData<DataType>
{
public:
	using Type = DataType;
	using OnReceiveProcType = void(const DataType& value);

	OSSIAN_SERVICE_SETUP(IODataImpl()) = default;
	~IODataImpl()                      = default;
	IODataImpl(const IODataImpl&)      = delete;

	IODataImpl(IODataImpl&& listener) noexcept
	{
		*this = std::move(listener);
	}

	auto operator=(const IODataImpl&) -> IODataImpl& = delete;

	auto operator=(IODataImpl&& listener) noexcept -> IODataImpl&
	{
		{
			std::lock_guard<Mutex> guard{m_Mutex};
			m_Payload = std::move(listener.m_Payload);
		}
		m_Mutex    = std::move(listener.m_Mutex);
		m_OnChange = std::move(m_OnChange);
		return *this;
	}

	auto Set(const DataType& value) noexcept -> void override
	{
		m_Mutex.lock();
		m_Payload     = value;
		m_RefreshFlag = true;
		m_Mutex.unlock();
		m_ConditionVariable.notify_one();
		m_OnChange(value);
	}

	auto Get() noexcept -> DataType override
	{
		std::lock_guard<Mutex> guard{m_Mutex};
		return m_Payload;
	}

	auto GetRef() const noexcept -> const DataType& override
	{
		return m_Payload;
	}

	auto WaitNextValue() -> DataType override
	{
		std::unique_lock<Mutex> lk{m_Mutex};
		m_RefreshFlag = false;
		m_ConditionVariable.wait(lk, [this]() { return m_RefreshFlag; });
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

	auto AddOnChange(std::function<OnReceiveProcType> callback) -> void override
	{
		m_OnChange = [callback, onChange = m_OnChange](const DataType& value)
		{
			onChange(value);
			callback(value);
		};
	}

private:
	DataType m_Payload = {};
	Mutex m_Mutex;
	std::function<OnReceiveProcType> m_OnChange = [](const DataType&)
	{
	};
	typename ConditionVariable<Mutex>::Type m_ConditionVariable;
	bool m_RefreshFlag;
};

/**
 * @brief Service builder of IOData.
 * Automatically add service to dependency injection module.
 * @tparam Mutex The mutex type.
 * @tparam DataModelTypes The types of payload data.
 */
template <typename Mutex, typename ...DataModelTypes>
class IODataServiceBuilder
{
	ApplicationBuilder& m_AppBuilder;
public:
	IODataServiceBuilder(ApplicationBuilder& appBuilder): m_AppBuilder(appBuilder)
	{
		std::make_tuple(std::move((appBuilder.template AddService<IOData<DataModelTypes>,
		                                                          IODataImpl<DataModelTypes, Mutex>>()))...);
	}
};


template <typename Mutex, typename ...DataModelTypes>
using IODataBuilder = CustomBuilder<IODataServiceBuilder<Mutex, DataModelTypes...>>;
} // ossian

#endif // OSSIAN_CORE_IODATA
