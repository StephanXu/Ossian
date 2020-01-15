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
#include "Pipeline.hpp"
#include "IOTypes.hpp"
#include "Service.hpp"
#include "DI.hpp"

namespace ossian
{
namespace IOAP
{
/**
 * @brief 延迟实例化函数
 * 部分需要批量存储实例化结果的依赖，在 ApplicationBuilder 被记录并传入 Dispatcher 中
 * @tparam RetT 实例化结果
 */
template <typename RetT>
using Realizer = std::function<RetT(DI::Injector &)>;

// 位于 Config.hpp
class ApplicationBuilder;

/**
 * @brief 分发器类型
 * 分发器类型应当由 ApplicationBuilder 创建
 */
class Dispatcher
{
    friend class ApplicationBuilder;

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
    explicit Dispatcher(DI::Injector &&injector,
                        std::vector<Realizer<std::tuple<std::type_index, Pipeline *>>> &pipelineRealizer,
                        std::vector<Realizer<BaseInputAdapter *>> &inputAdapterRealizer);

    ThreadPool m_ThreadPool;
    std::vector<std::tuple<std::type_index, Pipeline *>> m_Pipelines;
    std::vector<BaseInputAdapter *> m_InputAdapters;
    DI::Injector m_Injector;
};
} // namespace IOAP
} // namespace ossian
#endif // OSSIAN_CORE_DISPATCHER