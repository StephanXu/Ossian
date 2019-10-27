
#include "nv/Dispatcher.hpp"
#include <chrono>
#include <iostream>

namespace NautilusVision
{
namespace IOAP
{
Dispatcher::Dispatcher(DI::Injector &&injector,
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

void Dispatcher::Run()
{
    auto pureTick = std::chrono::system_clock::now();
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
                              << std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now() - pureTick)
                                     .count()
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
} // namespace IOAP
} // namespace NautilusVision