
#include "nv/Dispatcher.hpp"

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
    // pureTick = cv::getTickCount(); //< [注意]：测试代码
    while (true)
    {
        for (auto &&inputAdapter : m_InputAdapters)
        {
            auto input = inputAdapter->GetInput();
            if (!input)
            {
                // if (m_ThreadPool.Empty()) //< [注意]：测试代码
                // {
                //     std::cout << "Time:"
                //               << (cv::getTickCount() - pureTick) / cv::getTickFrequency()
                //               << std::endl;
                //     return;
                // }
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