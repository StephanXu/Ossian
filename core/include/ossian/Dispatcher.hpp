/**
 * @file Dispatcher.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 调度器相关实现
 * @version 0.1
 * @date 2019-10-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef OSSIAN_CORE_DISPATCHER
#define OSSIAN_CORE_DISPATCHER

#include <vector>
#include <memory>
#include <tuple>
#include <future>

#include "MultiThread.hpp"
#include "DI.hpp"

namespace ossian
{

// 位于 Config.hpp
class ApplicationBuilder;

/**
 * @brief 延迟实例化函数
 * 部分需要批量存储实例化结果的依赖，在 ApplicationBuilder 被记录并传入 Dispatcher 中
 * @tparam RetT 实例化结果
 */
template <typename RetT>
using Realizer = std::function<RetT(DI::Injector &)>;

/**
 * @brief 分发器类型
 * 分发器类型应当由 ApplicationBuilder 创建
 */
class Dispatcher
{
    friend class ossian::ApplicationBuilder;

public:
    Dispatcher() = delete;
    Dispatcher(const Dispatcher &dispatcher) = delete;
	Dispatcher(const Dispatcher&& dispatcher) = delete;

    void Run();

private:
    /**
     * @brief Construct a new Dispatcher object
     * 创建一个分发器对象
     * @param injector 实例化器
     * @param pipelineRealizer 管道延迟实例化函数
     * @param inputAdapterRealizer 输入适配器实例化函数
     */
    explicit Dispatcher(DI::Injector &&injector);

    ThreadPool m_ThreadPool;
    DI::Injector m_Injector;
};

} // namespace ossian

#endif // OSSIAN_CORE_DISPATCHER