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

#include <type_traits>

namespace NautilusVision
{

/*
IOAP 模型（输入->动作1->...->动作n->输出）
*/

class IExecutable
{
public:
    virtual void Process(BaseInputData &refInput) = 0;
    virtual bool IsSkip(const BaseStatus &refStatus) = 0;
};

class Pipeline
{
public:
    Pipeline(BaseStatus &status, BaseInputAdapter &refInputAdapter, std::vector<IExecutable *> &&actions)
        : m_Actions(std::move(actions)),
          m_Status(status),
          m_InputAdapter(refInputAdapter)
    {
    }

    void ProcessTask()
    {
        auto input = m_InputAdapter.GetInput();
        for (auto &&action : m_Actions)
        {
            if (!action->IsSkip(m_Status))
            {
                action->Process(*input);
            }
        }
        return;
    }

    template <typename ActionType, typename... Args>
    void RegisterAction(Args... args)
    {
        static_assert(std::is_base_of<IExecutable, ActionType>::value, "ActionType should derived from IExecutable");
        m_Actions.emplace_back(std::make_shared<ActionType>(std::forward<Args>(args)...));
    }

private:
    std::vector<IExecutable *> m_Actions;

    BaseStatus &m_Status;
    BaseInputAdapter &m_InputAdapter;
};

template <typename StatusType, typename InputAdapterType, typename... Args>
std::unique_ptr<Pipeline> CreatePipeline(StatusType *status, InputAdapterType *adapter, Args *... actions)
{
    static_assert(std::is_base_of<BaseStatus, StatusType>::value, "StatusType should derived from BaseStatus");
    static_assert(std::is_base_of<BaseInputAdapter, InputAdapterType>::value,
                  "InputAdapterType should derived from BaseInputAdapter");
    return std::make_unique<Pipeline>(*status, *adapter, std::vector<IExecutable *>{actions...});
}

} // namespace NautilusVision

#endif