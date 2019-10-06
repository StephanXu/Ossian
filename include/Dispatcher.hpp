
#ifndef DISPATCHER_HPP
#define DISPATCHER_HPP

#include <vector>
#include <memory>
#include <tuple>

#include "MultiThread.hpp"
#include "Pipeline.hpp"
#include "IOTypes.hpp"

namespace NautilusVision
{

class Dispatcher
{
public:
    template <typename StatusType>
    void InitializeStatus()
    {
        static_assert(std::is_base_of<BaseStatus, StatusType>::value,
                      "StatusType should derived from BaseStatus");
        m_Status.reset(new StatusType);
    }

    template <typename InputAdapter, typename... Args>
    void Register(std::initializer_list<Pipeline*> pipelines, Args... args)
    {
        static_assert(std::is_base_of<BaseInputAdapter, InputAdapter>::value,
                      "PipelineType should derived from BasePipeline");
        m_Pipelines.push_back(std::make_tuple(
            std::shared_ptr<BaseInputAdapter>(new InputAdapter{std::forward<Args>(args)...}),
            std::vector<Pipeline*>(pipelines)));
    }

    void Run()
    {
        while (true)
        {
            for (auto &&item : m_Pipelines)
            {
                auto input = std::get<0>(item)->GetInput();
                for (auto &&pipeline : std::get<1>(item))
                {
                    pipeline->ProcessTask(*input);
                    // m_ThreadPool.AddTask(&Pipeline::ProcessTask<BaseInputData>,
                    //                      pipeline.get(),
                    //                      *input);
                }
            }
        }
    }

private:
    ThreadPool m_ThreadPool;
    std::vector<std::tuple<std::shared_ptr<BaseInputAdapter>,
                           std::vector<Pipeline*>>>
        m_Pipelines;
    std::unique_ptr<BaseStatus> m_Status;
};

} // namespace NautilusVision
#endif