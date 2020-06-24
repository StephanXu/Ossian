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

#include "DI.hpp"
#include "Dispatcher.hpp"
#include "Factory.hpp"
#include "Pipeline.hpp"

namespace ossian
{
class ApplicationBuilder;

class IStartup
{
public:
	virtual ~IStartup() = default;
	virtual auto ConfigServices(ApplicationBuilder& app) -> void = 0;
	virtual auto ConfigPipeline(ApplicationBuilder& app) -> void = 0;
};

template <typename BuilderType>
class CustomBuilder
{
public:
	using _OssianServiceBuilderType = BuilderType;
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
		SPDLOG_TRACE("Register DI: {}\t{}", typeid(InterfaceType).name(), typeid(ServiceType).name());
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
using IsNeedBaseServiceBuilder = std::is_constructible<typename ServiceType::_OssianServiceBuilderType,
                                                       ApplicationBuilder&,
                                                       BaseServiceBuilder<InterfaceType, ServiceType>&>;

template <class ServiceType>
using IsValidCustomServiceBuilder = std::is_base_of<
	CustomBuilder<typename ServiceType::_OssianServiceBuilderType>, ServiceType>;

/**
 * Custom builder without BaseServiceBuilder
 */
template <class InterfaceType,
          class ServiceType>
class ServiceBuilder<InterfaceType,
                     ServiceType,
                     std::enable_if_t<IsValidCustomServiceBuilder<ServiceType>::value &&
                                      !IsNeedBaseServiceBuilder<InterfaceType, ServiceType>::value>>
	: BaseServiceBuilder<InterfaceType, ServiceType>, public ServiceType::_OssianServiceBuilderType
{
public:
	ServiceBuilder(ApplicationBuilder& DIConfig,
	               std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(DIConfig, configureProc)
		  , ServiceType::_OssianServiceBuilderType(DIConfig)
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
	: BaseServiceBuilder<InterfaceType, ServiceType>, public ServiceType::_OssianServiceBuilderType
{
public:
	ServiceBuilder(ApplicationBuilder& DIConfig,
	               std::function<void(ServiceType&)> configureProc)
		: BaseServiceBuilder<InterfaceType, ServiceType>(DIConfig, configureProc)
		  , ServiceType::_OssianServiceBuilderType(DIConfig, *this)
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

	template <typename TStartup, class...Args>
	auto UseStartup(Args ...args) -> ApplicationBuilder&
	{
		static_assert(std::is_base_of<IStartup, TStartup>::value, "TStartup should derived from ossian::IStartup.");
		m_Startup = std::make_unique<TStartup>(args...);
		m_Startup->ConfigServices(*this);
		m_Startup->ConfigPipeline(*this);
		return *this;
	}

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
	 * @brief 注册一项执行服务
	 * @tparam ServiceType 服务类型
	 */
	template <typename ExecutableType>
	auto AddExecutable(std::function<void(ExecutableType&)> configProc = [](ExecutableType&)
	{
	})
	{
		static_assert(std::is_base_of<IExecutable, ExecutableType>::value,
			"ExecutableType should derived from ossian::IExecutable");
		return AddService<IExecutable, ExecutableType>(configProc);
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
	 * @brief 实例化所有依赖并创建分发器
	 * 在实例化后 ApplicationBuilder 对象则可以被析构
	 * @return Dispatcher
	 */
	Dispatcher Realization()
	{
		SPDLOG_TRACE("Realization begin");
		return Dispatcher(m_DIConfig.BuildInjector());
	}

private:
	DI::DIConfiguration m_DIConfig;
	std::unique_ptr<IStartup> m_Startup;
};
} // namespace ossian

#endif // OSSIAN_CORE_CONFIG
