#ifndef OSSIAN_CORE_IODATA
#define OSSIAN_CORE_IODATA

#include "Factory.hpp"
#include "ApplicationBuilder.hpp"

#include <mutex>
#include <typeindex>

namespace ossian
{
template <typename DataType>
class IOData
{
	using OnReceiveProcType = void(const DataType& value);
public:
	virtual ~IOData() = default;
	virtual auto Set(const DataType& value) noexcept -> void = 0;
	virtual auto Get() noexcept -> DataType = 0;
	virtual auto GetRef() const noexcept -> const DataType& = 0;
	virtual auto Lock() -> void = 0;
	virtual auto UnLock() -> void = 0;
	virtual auto TryLock() noexcept -> bool = 0;
	virtual auto TypeIndex() noexcept -> std::type_index = 0;
	virtual auto AddOnChange(std::function<OnReceiveProcType> callback) -> void = 0;
};

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
		m_Payload = value;
		m_Mutex.unlock();
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
	std::function<OnReceiveProcType> m_OnChange = [](const DataType&){};
};

template <typename Mutex, typename ...DataModelTypes>
class IODataServiceBuilder
{
	template <int N, typename... Ts>
	using NThTypeOf = typename std::tuple_element<N, std::tuple<Ts...>>::type;

	ApplicationBuilder& m_AppBuilder;
public:
	IODataServiceBuilder(ApplicationBuilder& appBuilder): m_AppBuilder(appBuilder)
	{
		std::make_tuple(std::move((appBuilder.template AddService<IOData<DataModelTypes>,
		                                                          IODataImpl<DataModelTypes, Mutex>>()))...);
	}

	template <typename DataModelType, size_t ...Index>
	auto RegisterHelper()
	{
	}
};


template <typename Mutex, typename ...DataModelTypes>
using IODataBuilder = CustomBuilder<IODataServiceBuilder<Mutex, DataModelTypes...>>;
} // ossian

#endif // OSSIAN_CORE_IODATA
