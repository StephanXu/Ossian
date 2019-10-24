/**
 * @file Pipeline.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 处理管道相关支持
 * @version 0.1
 * @date 2019-10-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include "IOTypes.hpp"
#include "Service.hpp"

namespace NautilusVision
{

/**
 * @brief 执行体
 * 执行体定义了可执行的实例，可以被执行，并且提供一个可跳过的判断方法
 */
class IExecutable
{
public:
    virtual void Process(BaseInputData *refInput) = 0;
    virtual bool IsSkip(const BaseStatus &refStatus) = 0;
};

/**
 * @brief 管道实现
 * 管道是被调度器周期性调用的，接受特定输入的集合。其下包含若干个动作，以顺序的方式进行。
 */
class Pipeline
{
public:
    /**
     * @brief 创建一个管道实例
     * 
     * @param status 状态对象
     * @param actions 动作实例（有序）
     */
    Pipeline(BaseStatus &status, std::vector<IExecutable *> &&actions)
        : m_Actions(std::move(actions)),
          m_Status(status)
    {
    }
    ~Pipeline()
    {
    }

    /**
     * @brief 运行管道
     * @param inputData 输入数据，输入数据格式应当在注册时指定，不能为空
     */
    void ProcessTask(std::shared_ptr<BaseInputData> inputData)
    {
        if (!inputData)
        {
            throw std::runtime_error("inputData should not be null");
        }
        for (auto &&action : m_Actions)
        {
            if (!action->IsSkip(m_Status))
            {
                action->Process(inputData.get());
            }
        }
        return;
    }

private:
    std::vector<IExecutable *> m_Actions;
    BaseStatus &m_Status;
};

/**
 * @brief Create a Pipeline object
 * 生成一个管道实例
 * @tparam StatusType 状态类型
 * @tparam Args 动作类型
 * @param status 状态
 * @param actions 所有动作
 * @return std::unique_ptr<Pipeline> 管道实例
 */
template <typename StatusType, typename... Args>
std::unique_ptr<Pipeline> CreatePipeline(StatusType *status, Args *... actions)
{
    static_assert(std::is_base_of<BaseStatus, StatusType>::value, "StatusType should derived from BaseStatus");
    return std::make_unique<Pipeline>(*status, std::vector<IExecutable *>{actions...});
}

} // namespace NautilusVision

#endif