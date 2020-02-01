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

#include "Configuration.hpp"
#include "DI.hpp"
#include "Dispatcher.hpp"
#include "Factory.hpp"

namespace ossian
{
namespace IOAP
{

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
	 * @brief 注册一个状态对象
	 * @tparam StatusType 状态类型
	 */
	template <typename StatusType>
	ApplicationBuilder& RegisterStatusType()
	{
		m_DIConfig.Add(CreateStatus<StatusType>);
		return *this;
	}

	/**
	 * @brief 注册一项输入服务
	 * @tparam InputAdapterType 服务类型
	 */
	template <typename InputAdapterType>
	ApplicationBuilder& AddInputAdapter()
	{
		AddService<InputAdapterType>();
		m_InputAdapterRealizer.emplace_back(
			[](DI::Injector& injector) { return injector.GetInstance<InputAdapterType>(); });
		return *this;
	}

	/**
	 * @brief 注册一条管道
	 * 注意：目前仅支持注册一条管道
	 * @tparam StatusType 状态类型
	 * @tparam InputType 输入类型
	 * @tparam ActionTypes 动作类型
	 */
	template <typename StatusType, typename InputType, typename... ActionTypes>
	ApplicationBuilder& RegisterPipeline()
	{
		m_DIConfig.Add(CreatePipeline<StatusType, ActionTypes...>);
		m_PipelineRealizer.emplace_back(
			[](DI::Injector& injector)
			{
				return std::make_tuple(std::type_index(typeid(InputType)),
									   injector.GetInstance<Pipeline>());
			});
		return *this;
	}

	/**
	 * @brief 注册一项服务
	 * @tparam ServiceType 服务类型
	 */
	template<typename T>
	ApplicationBuilder& AddService(std::function<void(T&)> configProc=[](T&){})
	{
		//m_DIConfig.Add(Factory::CreateGeneralService<T, Deps...>);
		m_DIConfig.Add(InstanceFactory<typename T::_OssianServiceInjectorType>::Value(configProc));
		//return ServiceBuilder<T>(this);
		return *this;
	}

	/**
	 * @brief 利用工厂函数手动注册
	 * 手动注册依赖
	 * @tparam InstanceType
	 * @tparam Deleter
	 * @tparam Deps
	 * @param instanceFactory
	 */
	template <class InstanceType, class Deleter, class... Deps>
	ApplicationBuilder& Add(DI::InstanceFactoryNativeFunction<InstanceType, Deleter, Deps...> instanceFactory)
	{
		m_DIConfig.Add(instanceFactory);
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
	template <class InstanceType, class Deleter, class... Deps>
	ApplicationBuilder& Add(DI::InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory)
	{
		m_DIConfig.Add(instanceFactory);
		return *this;
	}

	/**
	 * @brief 实例化所有依赖并创建分发器
	 * 在实例化后 ApplicationBuilder 对象则可以被析构
	 * @return Dispatcher
	 */
	Dispatcher Realization()
	{
		spdlog::info("Initialize configuration");
		return Dispatcher(m_DIConfig.BuildInjector(), m_PipelineRealizer, m_InputAdapterRealizer);
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

private:
	DI::DIConfiguration m_DIConfig;

	// 用于获得实例化后的Pipeline，但此处框架仅支持一个Pipeline，使用vector是为了之后支持多Pipeline做准备
	std::vector<Realizer<std::tuple<std::type_index, Pipeline*>>> m_PipelineRealizer;

	// 用于获得实例化后的InputAdapter
	std::vector<Realizer<BaseInputAdapter*>> m_InputAdapterRealizer;
};
} // namespace IOAP
} // namespace ossian

#endif // OSSIAN_CORE_CONFIG