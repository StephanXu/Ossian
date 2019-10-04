
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

    template <typename InputType, typename... Pipelines>
    void RegisterPipeline(Pipelines... pipelines)
    {
        static_assert(std::is_base_of<BaseAdapter, InputType>::value,
                      "PipelineType should derived from BasePipeline");
        vec.push_back(std::make_tuple(std::shared_ptr<InputType>(new InputType),
                                      {pipelines...}));
    }

private:
    ThreadPool m_ThreadPool;
    std::vector<std::tuple<std::shared_ptr<BaseAdapter>,
                           std::vector<std::shared_ptr<BasePipeline>>>>
        m_Pipelines;
    std::unique_ptr<BaseStatus> m_Status;
};

} // namespace NautilusVision
#endif