
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

    template <typename InputAdapter, typename... Pipelines>
    void Register(Pipelines... pipelines)
    {
        static_assert(std::is_base_of<BaseAdapter, InputAdapter>::value,
                      "PipelineType should derived from BasePipeline");
        vec.push_back(std::make_tuple(
            std::shared_ptr<InputAdapter>(new InputAdapter),
            {pipelines...}));
    }

    void Process()
    {
        for (auto &&item : m_Pipelines)
        {
            auto input = std::get<0>(item)->GetInput();
            for (auto &&pipeline : std::get<1>(item))
            {
                pipeline->ProcessTask(*input);
            }
        }
    }

private:
    ThreadPool m_ThreadPool;
    std::vector<std::tuple<std::shared_ptr<BaseInputAdapter>,
                           std::vector<std::shared_ptr<Pipeline>>>>
        m_Pipelines;
    std::unique_ptr<BaseStatus> m_Status;
};

} // namespace NautilusVision
#endif