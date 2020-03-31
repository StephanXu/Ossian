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

#include "DI.hpp"
#include "Dispatcher.hpp"
#include "Factory.hpp"

namespace ossian
{
class ApplicationBuilder;

template <typename BuilderType>
class CustomBuilder
{
public:
	using OssianServiceBuilderType = BuilderType;
};

class DefaultServiceBuilder
{
public:
	DefaultServiceBuilder(ApplicationBuilder& DIConfig)
	{
	}
};

using DefaultBuilder = CustomBuilder<DefaultServiceBuilder>;

template <class InterfaceType,
          class ServiceType = InterfaceType>
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

	//BaseServiceBuilder(const BaseServiceBuilder<InterfaceType, ServiceType>&) = delete;

	virtual ~BaseServiceBuilder()
	{
		spdlog::trace("Register DI: {}\t{}", typeid(InterfaceType).name(), typeid(ServiceType).name());
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

template <class InterfaceType,
          class ServiceType = InterfaceType,
          typename Enabled = void>
class ServiceBuilder : BaseServiceBuilder<InterfaceType, ServiceType>
{
public:
	ServiceBuilder(ApplicationBuilder& DIConfig,
	               std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(DIConfig, configureProc)
	{
	}
};


template <class InterfaceType, class ServiceType>
using IsNeedBaseServiceBuilder = std::is_constructible<typename ServiceType::OssianServiceBuilderType,
                                                       ApplicationBuilder&,
                                                       BaseServiceBuilder<InterfaceType, ServiceType>&>;

template <class ServiceType>
using IsValidCustomServiceBuilder = std::is_base_of<
	CustomBuilder<typename ServiceType::OssianServiceBuilderType>, ServiceType>;

/**
 * Custom builder without BaseServiceBuilder
 */
template <class InterfaceType,
          class ServiceType>
class ServiceBuilder<InterfaceType,
                     ServiceType,
                     std::enable_if_t<IsValidCustomServiceBuilder<ServiceType>::value &&
                                      !IsNeedBaseServiceBuilder<InterfaceType, ServiceType>::value>>
	: BaseServiceBuilder<InterfaceType, ServiceType>, public ServiceType::OssianServiceBuilderType
{
public:
	ServiceBuilder(ApplicationBuilder& DIConfig,
	               std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(DIConfig, configureProc)
		  , ServiceType::OssianServiceBuilderType(DIConfig)
	{
	}
};

/**
 * Custom builder with BaseServiceBuilder
 */
template <class InterfaceType,
          class ServiceType>
class ServiceBuilder<InterfaceType,
                     ServiceType,
                     std::enable_if_t<IsValidCustomServiceBuilder<ServiceType>::value &&
                                      IsNeedBaseServiceBuilder<InterfaceType, ServiceType>::value>>
	: BaseServiceBuilder<InterfaceType, ServiceType>, public ServiceType::OssianServiceBuilderType
{
public:
	ServiceBuilder(ApplicationBuilder& DIConfig,
	               std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(DIConfig, configureProc)
		  , ServiceType::OssianServiceBuilderType(DIConfig, *this)
	{
	}
};

/**
 * @brief 配置器实现
 * 通过此类进行配置并生成 Dispatcher 对象
 */
class ApplicationBuilder
{
public:
	ApplicationBuilder()                                      = default;
	ApplicationBuilder(const ApplicationBuilder& dispatcher)  = delete;
	ApplicationBuilder(const ApplicationBuilder&& dispatcher) = delete;

	/**
	 * @brief 注册一项服务
	 * @tparam ServiceType 服务类型
	 */
	template <typename InterfaceType, typename ServiceType>
	auto AddService(std::function<void(ServiceType&)> configProc = [](ServiceType&)
	{
	})
	{
		return ServiceBuilder<InterfaceType, ServiceType>(*this, configProc);
	}

	template <typename ServiceType>
	auto AddService(std::function<void(ServiceType&)> configProc = [](ServiceType&)
	{
	})
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
	template <class InterfaceType, class InstanceType, class Deleter, class... Deps>
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
	template <class InterfaceType, class InstanceType, class Deleter, class... Deps>
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
		spdlog::set_pattern("[%Y-%m-%dT%T.%e%z] [%-5t] %^[%l]%$ %v");
		spdlog::set_level(spdlog::level::trace);
		return *this;
	}

	/**
	 * @brief 实例化所有依赖并创建分发器
	 * 在实例化后 ApplicationBuilder 对象则可以被析构
	 * @return Dispatcher
	 */
	Dispatcher Realization()
	{
		spdlog::trace("Realization begin");
		return Dispatcher(m_DIConfig.BuildInjector());
	}

private:
	DI::DIConfiguration m_DIConfig;
};
} // namespace ossian

#endif // OSSIAN_CORE_CONFIG
