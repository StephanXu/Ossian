
#ifndef DISPATCHER_HPP
#define DISPATCHER_HPP

#include <vector>
#include <memory>
#include <tuple>
#include <future>

#include "MultiThread.hpp"
#include "Pipeline.hpp"
#include "IOTypes.hpp"
#include "Service.hpp"
#include "DI.hpp"

namespace NautilusVision
{

/**
 * @brief 分发器类型
 * 
 */
class Dispatcher
{
public:
    Dispatcher() = default;
    Dispatcher(const Dispatcher &dispatcher) = delete;
    Dispatcher(const Dispatcher &&dispatcher) = delete;
    Dispatcher(DI::Injector &&injector,
               std::vector<std::packaged_task<Pipeline *(DI::Injector &)>> &pipelineRealizer)
    {
        for (auto &&realizer : pipelineRealizer)
        {
            realizer(injector);
            m_Pipelines.push_back(realizer.get_future().get());
        }
    }

    void Run()
    {
        pureTick = cv::getTickCount(); //< [注意]：测试代码
        while (true)
        {
            for (auto &&pipeline : m_Pipelines)
            {
                // if (!input)
                // {
                //     if (m_ThreadPool.Empty()) //< [注意]：测试代码
                //     {
                //         std::cout << "Time:"
                //                   << (cv::getTickCount() - pureTick) / cv::getTickFrequency()
                //                   << std::endl;
                //         return;
                //     }
                //     continue;
                // }

                // pipeline->ProcessTask();
                m_ThreadPool.AddTask(&Pipeline::ProcessTask, pipeline);
            }
        }
    }

private:
    ThreadPool m_ThreadPool;
    std::vector<Pipeline *> m_Pipelines;
    long long pureTick; //< [注意]：测试代码
};

class ApplicationBuilder
{
public:
    ApplicationBuilder() = default;
    ApplicationBuilder(const ApplicationBuilder &dispatcher) = delete;
    ApplicationBuilder(const ApplicationBuilder &&dispatcher) = delete;

    template <typename StatusType>
    void RegisterStatusType()
    {
        m_DIConfig.Add(CreateStatus<StatusType>);
        m_StatusRealizer = std::make_unique([](DI::Injector &injector) { return injector.GetInstance<StatusType>(); })
    }

    template <typename ServiceType, typename... Args>
    void RegisterService(Args... args)
    {
        m_DIConfig.Add(CreateService<ServiceType, Args...>);
    }

    //now only support single pipeline.
    template <typename StatusType, typename InputAdapterType>
    void RegisterPipeline()
    {
        m_DIConfig.Add(CreatePipeline<StatusType, InputAdapterType>);
        m_PipelineRealizer.emplace_back([](DI::Injector &injector) { return injector.GetInstance<Pipeline>() });
    }

    Dispatcher Realization()
    {
        return Dispatcher(m_DIConfig.BuildInjector(), m_PipelineRealizer);
    }

private:
    DI::DIConfiguration m_DIConfig;

    // 用于获得实例化后的Pipeline，但此处框架仅支持一个Pipeline，使用vector是为了之后支持多Pipeline做准备
    std::vector<std::packaged_task<Pipeline *(DI::Injector &)>> m_PipelineRealizer;

    // 用于获得实例化后的Status
    std::unique_ptr<std::packaged_task<BaseStatus *(DI::Injector &)>> m_StatusRealizer;
};

} // namespace NautilusVision
#endif