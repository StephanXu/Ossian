
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

template <typename RetT>
using Realizer = std::packaged_task<RetT(DI::Injector &)>;

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
               std::vector<Realizer<std::tuple<std::type_index, Pipeline *>>> &pipelineRealizer,
               std::vector<Realizer<BaseInputAdapter *>> &inputAdapterRealizer)
        : m_Injector(std::move(injector))
    {
        for (auto &&realizer : pipelineRealizer)
        {
            realizer(m_Injector);
            m_Pipelines.push_back(realizer.get_future().get());
        }
        for (auto &&realizer : inputAdapterRealizer)
        {
            realizer(m_Injector);
            m_InputAdapters.push_back(realizer.get_future().get());
        }
    }

    void Run()
    {
        pureTick = cv::getTickCount(); //< [注意]：测试代码
        while (true)
        {
            for (auto &&inputAdapter : m_InputAdapters)
            {
                auto input = inputAdapter->GetInput();
                if (!input)
                {
                    if (m_ThreadPool.Empty()) //< [注意]：测试代码
                    {
                        std::cout << "Time:"
                                  << (cv::getTickCount() - pureTick) / cv::getTickFrequency()
                                  << std::endl;
                        return;
                    }
                    continue;
                }
                for (auto &&pipePack : m_Pipelines)
                {
                    if (inputAdapter->GetInputTypeIndex() == std::get<0>(pipePack))
                    {
                        m_ThreadPool.AddTask(&Pipeline::ProcessTask, std::get<1>(pipePack), input);
                    }
                    // pipeline->ProcessTask();
                }
            }
        }
    }

private:
    ThreadPool m_ThreadPool;
    std::vector<std::tuple<std::type_index, Pipeline *>> m_Pipelines;
    std::vector<BaseInputAdapter *> m_InputAdapters;
    DI::Injector m_Injector;

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
    }

    template <typename ServiceType>
    void RegisterService()
    {
        m_DIConfig.Add(CreateService<ServiceType>);
        m_InputAdapterRealizer.emplace_back([](DI::Injector &injector) { return injector.GetInstance<ServiceType>(); });
    }

    //now only support single pipeline.
    template <typename StatusType, typename InputType, typename... Args>
    void RegisterPipeline()
    {
        m_DIConfig.Add(CreatePipeline<StatusType, Args...>);
        m_PipelineRealizer.emplace_back([](DI::Injector &injector) {
            return std::make_tuple(std::type_index(typeid(InputType)),
                                   injector.GetInstance<Pipeline>());
        });
    }

    template <class InstanceType, class Deleter, class... Deps>
    using InstanceFactoryFunction = std::unique_ptr<InstanceType, Deleter> (*)(Deps *...);

    template <class InstanceType, class Deleter, class... Deps>
    void Register(InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory)
    {
        m_DIConfig.Add(instanceFactory);
    }

    Dispatcher Realization()
    {
        return Dispatcher(m_DIConfig.BuildInjector(), m_PipelineRealizer, m_InputAdapterRealizer);
    }

private:
    DI::DIConfiguration m_DIConfig;

    // 用于获得实例化后的Pipeline，但此处框架仅支持一个Pipeline，使用vector是为了之后支持多Pipeline做准备
    std::vector<Realizer<std::tuple<std::type_index, Pipeline *>>> m_PipelineRealizer;

    // 用于获得实例化后的InputAdapter
    std::vector<Realizer<BaseInputAdapter *>> m_InputAdapterRealizer;
};

} // namespace NautilusVision
#endif