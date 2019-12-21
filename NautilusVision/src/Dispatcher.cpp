#include <spdlog/spdlog.h>

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
        m_Pipelines.push_back(realizer(m_Injector));
    }
    for (auto &&realizer : inputAdapterRealizer)
    {
        m_InputAdapters.push_back(realizer(m_Injector));
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
					spdlog::info("Time: {}",
								 std::chrono::duration_cast<std::chrono::milliseconds>(
									 std::chrono::system_clock::now() - pureTick).count());
					return;
				}
                continue;
            }
            for (auto &&pipePack : m_Pipelines)
            {
                if (inputAdapter->GetInputTypeIndex() == std::get<0>(pipePack))
                {
					//spdlog::info("Remain task: {}", m_ThreadPool.WaitingCount());
                    m_ThreadPool.AddTask(&Pipeline::ProcessTask, std::get<1>(pipePack), input);
					//std::get<1>(pipePack)->ProcessTask(input);
                }
            }
			std::this_thread::sleep_for(std::chrono::milliseconds(2)); //[ATTENTIOM]: God damn trick
        }
    }
}
} // namespace IOAP
} // namespace NautilusVision