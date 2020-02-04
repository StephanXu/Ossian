/**
 * @file Config.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 配置器的相关实现
 * @version 0.1
 * @date 2019-10-25
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef OSSIAN_CORE_CONFIG
#define OSSIAN_CORE_CONFIG

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <string>

#include "Service.hpp"
#include "DI.hpp"
#include "Dispatcher.hpp"

namespace ossian
{

class ApplicationBuilder;

template<
	class InterfaceType,
	class ServiceType = InterfaceType
>
class BaseServiceBuilder
{
public:
	BaseServiceBuilder(ApplicationBuilder& appBuilder,
					   std::function<void(ServiceType&)> configureProc) : m_AppBuilder(appBuilder),
		m_ConfigureProc(configureProc)
	{
		static_assert(std::is_base_of<InterfaceType, ServiceType>::value,
					  "ServiceType has to be derived from InterfaceType");
	}

	virtual ~BaseServiceBuilder()
	{
		m_AppBuilder.template Add<InterfaceType>(
			InstanceFactory<typename ServiceType::_OssianServiceInjectorType>::Value(m_ConfigureProc));
	}


	BaseServiceBuilder& AddConfig(std::function<void(ServiceType&)> configureProc)
	{
		m_ConfigureProc = [configureProc, oldConfigureProc = m_ConfigureProc](ServiceType& service)
		{
			oldConfigureProc(service);
			configureProc(service);
		};
		return *this;
	}

protected:
	ApplicationBuilder& m_AppBuilder;
	std::function<void(ServiceType&)> m_ConfigureProc;
};

template<
	class InterfaceType,
	class ServiceType = InterfaceType,
	typename CollectionFlag = std::true_type
>
class ServiceBuilder : BaseServiceBuilder<InterfaceType, ServiceType>
{
public:
	ServiceBuilder(ApplicationBuilder& DIConfig,
				   std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(DIConfig, configureProc)
	{
	}
};

template<
	class InterfaceType,
	class ServiceType
>
class ServiceBuilder<InterfaceType, ServiceType, typename std::is_base_of<IOAP::BaseInputAdapter, ServiceType>::type>
	: BaseServiceBuilder<InterfaceType, ServiceType>
{
public:
	ServiceBuilder(ApplicationBuilder& appBuilder,
				   std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(appBuilder, configureProc)
	{
	}

	auto AsInputAdapter() -> ServiceBuilder<InterfaceType, ServiceType,
		typename std::is_base_of<IOAP::BaseInputAdapter, ServiceType>::type>&
	{
		this->m_AppBuilder.template SetServiceAsInputAdapter<ServiceType>();
		return *this;
	}
};

/**
 * @brief 配置器实现
 * 通过此类进行配置并生成 Dispatcher 对象
 */
class ApplicationBuilder
{
public:
	ApplicationBuilder() = default;
	ApplicationBuilder(const ApplicationBuilder& dispatcher) = delete;
	ApplicationBuilder(const ApplicationBuilder&& dispatcher) = delete;

	/**
	 * @brief 注册一项输入服务
	 * @tparam InputAdapterType 服务类型
	 */
	template<typename InputAdapterType>
	ApplicationBuilder& AddInputAdapter()
	{
		AddService<InputAdapterType>();
		SetServiceAsInputAdapter<InputAdapterType>();
		return *this;
	}

	template<typename ServiceType>
	ApplicationBuilder& SetServiceAsInputAdapter()
	{
		m_InputAdapterRealizer.emplace_back(
			[](DI::Injector& injector) { return injector.GetInstance<ServiceType>(); });
		return *this;
	}

	/**
	 * @brief 注册一条管道
	 * 注意：目前仅支持注册一条管道
	 * @tparam StatusType 状态类型
	 * @tparam InputType 输入类型
	 * @tparam ActionTypes 动作类型
	 */
	template<typename StatusType, typename InputType, typename... ActionTypes>
	ApplicationBuilder& RegisterPipeline()
	{
		m_DIConfig.Add<IOAP::Pipeline>(IOAP::CreatePipeline<StatusType, ActionTypes...>);
		m_PipelineRealizer.emplace_back([](DI::Injector& injector)
										{
											return std::make_tuple(std::type_index(typeid(InputType)),
																   injector.GetInstance<IOAP::Pipeline>());
										});
		return *this;
	}

	/**
	 * @brief 注册一项服务
	 * @tparam ServiceType 服务类型
	 */
	template<typename InterfaceType, typename ServiceType>
	auto AddService(std::function<void(ServiceType&)> configProc = [](ServiceType&) {})
		-> ServiceBuilder<InterfaceType, ServiceType>
	{
		return ServiceBuilder<InterfaceType, ServiceType>(*this, configProc);
	}

	template<typename ServiceType>
	auto AddService(std::function<void(ServiceType&)> configProc = [](ServiceType&) {})
		-> ServiceBuilder<ServiceType, ServiceType>
	{
		return AddService<ServiceType, ServiceType>(configProc);
	}

	/**
	 * @brief 利用工厂函数手动注册
	 * 手动注册依赖
	 * @tparam InstanceType
	 * @tparam Deleter
	 * @tparam Deps
	 * @param instanceFactory
	 */
	template<class InterfaceType, class InstanceType, class Deleter, class... Deps>
	ApplicationBuilder& Add(DI::InstanceFactoryNativeFunction<InstanceType, Deleter, Deps...> instanceFactory)
	{
		m_DIConfig.Add<InterfaceType>(instanceFactory);
		return *this;
	}

	/**
	 * @brief 利用仿函数的工厂函数手动添加一项服务
	 * @tparam InstanceType 服务类型
	 * @tparam Deleter
	 * @tparam Deps 依赖
	 * @param instanceFactory 构造函数
	 * @return
	 */
	template<class InterfaceType, class InstanceType, class Deleter, class... Deps>
	ApplicationBuilder& Add(DI::InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory)
	{
		m_DIConfig.Add<InterfaceType>(instanceFactory);
		return *this;
	}

	/**
	 * @brief	初始化日志
	 * @author	Xu Zihan
	 * @date	2019/11/21
	 */
	ApplicationBuilder& InitLog()
	{
		const auto console = spdlog::stderr_color_mt("console");
		spdlog::set_default_logger(console);
		spdlog::set_pattern("[%T.%e] [%-5t] %^[%l]%$  %v");
		spdlog::set_level(spdlog::level::info);
		return *this;
	}

	/**
	 * @brief 实例化所有依赖并创建分发器
	 * 在实例化后 ApplicationBuilder 对象则可以被析构
	 * @return Dispatcher
	 */
	IOAP::Dispatcher Realization()
	{
		spdlog::info("Initialize configuration");
		return IOAP::Dispatcher(m_DIConfig.BuildInjector(), m_PipelineRealizer, m_InputAdapterRealizer);
	}

private:
	DI::DIConfiguration m_DIConfig;

	// 用于获得实例化后的Pipeline，但此处框架仅支持一个Pipeline，使用vector是为了之后支持多Pipeline做准备
	std::vector<IOAP::Realizer < std::tuple<std::type_index, IOAP::Pipeline*>>>
		m_PipelineRealizer;

	// 用于获得实例化后的InputAdapter
	std::vector<IOAP::Realizer < IOAP::BaseInputAdapter* >> m_InputAdapterRealizer;
};

} // namespace ossian

#endif // OSSIAN_CORE_CONFIG