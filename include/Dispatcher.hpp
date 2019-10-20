
#ifndef DISPATCHER_HPP
#define DISPATCHER_HPP

#include <vector>
#include <memory>
#include <tuple>

#include "MultiThread.hpp"
#include "Pipeline.hpp"
#include "IOTypes.hpp"
#include "Service.hpp"

namespace NautilusVision
{

/**
 * @brief 分发器类型
 * 
 */
class Dispatcher
{
public:
    template <typename StatusType>
    void InitializeStatus()
    {
        static_assert(std::is_base_of<BaseStatus, StatusType>::value, "StatusType should derived from BaseStatus");
        m_Status.reset(new StatusType);
    }

    template <typename InputAdapter, typename... Args>
    void Register(std::initializer_list<Pipeline *> pipelines, Args... args)
    {
        static_assert(std::is_base_of<BaseInputAdapter, InputAdapter>::value,
                      "PipelineType should derived from BasePipeline");
        m_Pipelines.push_back(std::make_shared<Pipeline>(m_Status,));
    }

    template <typename ServiceType, typename... Args>
    void RegisterService(Args... args)
    {
        static_assert(std::is_base_of<IService, ServiceType>::value, "ServiceType should derived from ServiceType");
        m_Services.push_back(std::make_shared<ServiceType>(std::forward<Args>(args)...));
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
                m_ThreadPool.AddTask(&Pipeline::ProcessTask, pipeline.get());
            }
        }
    }

private:
    ThreadPool m_ThreadPool;
    std::vector<std::shared_ptr<Pipeline>> m_Pipelines;
    std::vector<std::shared_ptr<IService>> m_Services;
    std::unique_ptr<BaseStatus> m_Status;

    long long pureTick; //< [注意]：测试代码
};

} // namespace NautilusVision
#endif