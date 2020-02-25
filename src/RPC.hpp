
#ifndef OSSIAN_RPC_HPP
#define OSSIAN_RPC_HPP

#include <signalrclient/signalr_value.h>
#include <signalrclient/hub_connection.h>
#include <spdlog/spdlog.h>

#include <type_traits>
#include <vector>
#include <string>
#include <functional>
#include <future>

class ValueConvert
{
	template<class...> struct conjunction : std::true_type { };
	template<class B1> struct conjunction<B1> : B1 { };
	template<class B1, class... Bn>
	struct conjunction<B1, Bn...>
		: std::conditional_t<bool(B1::value), conjunction<Bn...>, B1> {};

	template<class...> struct disjunction : std::false_type { };
	template<class B1> struct disjunction<B1> : B1 { };
	template<class B1, class... Bn>
	struct disjunction<B1, Bn...>
		: std::conditional_t<bool(B1::value), B1, disjunction<Bn...>> { };

	template <typename T>
	using is_plain_type = disjunction<
		std::is_same<T, float>,
		std::is_same<T, std::string>,
		std::is_same<T, bool>>;

	template <typename T>
	using is_vector = std::is_same<T, std::vector<
		typename T::value_type,
		typename T::allocator_type
		>>;

	template <typename T>
	using is_plain_vector = conjunction<
		is_vector<T>,
		is_plain_type<typename T::value_type>>;
public:

	// Convert type to value

	template<
		class T,
		std::enable_if_t<is_plain_type<T>::value, int> = 0
	>static auto TypeToValue(T n)->T
	{
		return n;
	}

	template<
		class T,
		std::enable_if_t<is_plain_vector<T>::value, int> = 0
	>static auto TypeToValue(T& n)->std::vector<signalr::value>
	{
		return std::vector<signalr::value>{ n.begin(), n.end() };
	}

	template<
		class T,
		std::enable_if_t<std::is_same<T, signalr::value>::value, int> = 0
	>static auto TypeToValue(T& n)->T&
	{
		return n;
	}

	// Convert value to type

	template<
		class T,
		std::enable_if_t<std::is_same<std::remove_cv_t<T>, bool>::value, int> = 0
	>static auto ValueToType(const signalr::value& n)
	{
		return n.as_bool();
	}

	template<
		class T,
		std::enable_if_t<std::is_same<std::remove_cv_t<T>, std::string>::value, int> = 0
	>static auto ValueToType(const signalr::value& n)->const T&
	{
		return n.as_string();
	}

	template<
		class T,
		std::enable_if_t<std::is_same<std::remove_cv_t<T>, double>::value, int> = 0
	>static auto ValueToType(const signalr::value& n)
	{
		return n.as_double();
	}

	template<
		class T,
		std::enable_if_t<is_plain_vector<std::remove_cv_t<T>>::value, int> = 0
	>static auto ValueToType(const signalr::value& n)->const std::vector<signalr::value>&
	{
		if (!n.is_array())
			throw std::bad_cast();
		return n.as_array();
	}

	template<
		class T,
		std::enable_if_t<std::is_same<std::remove_cv_t<T>, signalr::value>::value, int> = 0
	>static auto ValueToType(const signalr::value& n)->T&
	{
		return n;
	}
};

class RPCInvoker
{
public:
	using SignalRCallbackType = std::function<void(const signalr::value&, std::exception_ptr)>;

	template<typename RetT>
	using CallbackType = std::function<void(const RetT&, std::exception_ptr)>;

	template<class...Args>
	class InvokeBuilder
	{
		signalr::hub_connection& m_Connection;
		std::string m_MethodName;
		signalr::value m_Arguments;
		SignalRCallbackType m_Callback;
	public:
		InvokeBuilder(signalr::hub_connection& connection, const std::string methodName, Args...args)
			:m_Connection(connection)
			, m_MethodName(methodName)
			, m_Arguments(RPCInvoker::PrepareArguments(args...))
			, m_Callback([](const signalr::value&, std::exception_ptr) {})
		{
		}
		InvokeBuilder(const InvokeBuilder&) = delete;
		InvokeBuilder& operator=(const InvokeBuilder&) = delete;
		InvokeBuilder(InvokeBuilder&&) = delete;
		void operator=(InvokeBuilder&&) = delete;

		~InvokeBuilder()
		{
			RPCInvoker::InvokeHelper(m_Connection, m_MethodName, m_Arguments, m_Callback);
		}

		template<typename RetT>
		void Then(CallbackType<RetT> callback)
		{
			m_Callback = [callback](const signalr::value& value, std::exception_ptr exception)
			{
				callback(ValueConvert::ValueToType<RetT>(value), exception);
			};
			//m_Callback = callback;
		}
	};

	template <class...Args>
	static auto Invoke(signalr::hub_connection& connection,
					   const std::string methodName,
					   Args...args)
	{
		return InvokeBuilder<Args...>{connection, methodName, args...};
	}

	template<class...Args>
	static signalr::value PrepareArguments(Args...args)
	{
		return std::move(std::vector<signalr::value>{ValueConvert::TypeToValue(args)...});
	}

private:
	static void InvokeHelper(signalr::hub_connection& connection,
							 const std::string methodName,
							 const signalr::value& arguments,
							 const SignalRCallbackType callback) noexcept
	{
		connection.invoke(methodName, arguments, callback);
	}
};

class BaseHub
{
	signalr::hub_connection m_Connection;
	std::promise<void> m_Task;
	bool m_Valid;

	template<typename ...Args, size_t...Index>
	static auto ExtractArguments(std::vector<signalr::value> params, std::index_sequence<Index...>)
	{
		return std::make_tuple(ValueConvert::ValueToType<Args>(params[Index])...);
	}

protected:
	BaseHub(signalr::hub_connection&& connection)
		:m_Connection(std::move(connection))
		, m_Valid(false)
	{
	}

	auto Start()->void
	{
		std::promise<void> waitTask;
		m_Connection.start(
			[this, &waitTask](std::exception_ptr exception)
			{
				if (exception)
				{
					try
					{
						std::rethrow_exception(exception);
					}
					catch (const std::exception & ex)
					{
						spdlog::error("exception when starting connection: {}", ex.what());
					}
					m_Valid = false;
					m_Task.set_value();
					waitTask.set_value();
					return;
				}
				m_Valid = true;
				waitTask.set_value();
			});
		waitTask.get_future().get();
	}

	auto StartAsync()->void
	{
		m_Connection.start(
			[this](std::exception_ptr exception)
			{
				if (exception)
				{
					try
					{
						std::rethrow_exception(exception);
					}
					catch (const std::exception & ex)
					{
						spdlog::error("exception when starting connection: {}", ex.what());
					}
					m_Valid = false;
					m_Task.set_value();
					return;
				}
				m_Valid = true;
			});
	}

public:
	BaseHub() = delete;
	BaseHub(const BaseHub&) = delete;
	BaseHub(BaseHub&& hub) noexcept
		:m_Connection(std::move(hub.m_Connection))
		, m_Task(std::move(hub.m_Task))
		, m_Valid(hub.m_Valid)
	{
	}
	BaseHub& operator=(const BaseHub&) = delete;
	void operator=(BaseHub&& hub) noexcept
	{
		m_Connection = std::move(hub.m_Connection);
		m_Task = std::move(hub.m_Task);
		m_Valid = hub.m_Valid;
	}

	virtual ~BaseHub()
	{
		if (m_Connection.get_connection_state() == signalr::connection_state::connected)
		{
			m_Connection.stop(
				[this](std::exception_ptr exception)
				{
					try
					{
						if (exception)
						{
							std::rethrow_exception(exception);
						}
						spdlog::info("connection stopped successfully");
					}
					catch (const std::exception & e)
					{
						spdlog::error("exception when stopping connection: {}", e.what());
					}
					m_Task.set_value();
				});
			m_Task.get_future().get();
		}
	}

	auto Connection()->signalr::hub_connection& { return m_Connection; }

	auto IsValid()const { return m_Valid; }

	template<typename ...Args>
	auto Invoke(std::string methodName, Args ...args)
	{
		return RPCInvoker::Invoke(m_Connection, methodName, std::forward<Args>(args)...);
	}

	template<typename RetT, typename ...Args>
	auto On(std::string methodName, std::function<RetT(Args...)> callback)->void
	{
		m_Connection.on(
			methodName,
			[callback](const signalr::value& param)
			{
				std::vector<signalr::value> arguments = param.as_array();
				auto indexSeq = std::make_index_sequence<sizeof...(Args)>();
				std::tuple<Args...> parameters =
					ExtractArguments<Args...>(arguments, std::index_sequence_for<Args...>());
				std::apply(callback, parameters);
			});
	}

	template<typename RetT, typename ...Args>
	using NativeCallbackType = RetT(*)(Args...);

	template<typename RetT, typename ...Args>
	auto On(std::string methodName, NativeCallbackType<RetT, Args...> callback)
	{
		On(methodName, std::function<RetT(Args...)>{callback});
	}
};

#define HUB_REGISTER_CALLBACK(callbackName) BaseHub::On(#callbackName, callbackName)

#endif // OSSIAN_RPC_HPP