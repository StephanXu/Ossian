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
#include <type_traits>

namespace NautilusVision
{

/*
IOAP 模型（输入->动作1->...->动作n->输出）
*/

class IExecutable
{
public:
    virtual void Process(BaseInputData &refInput,
                         BaseOutputData &outOutput) = 0;
    virtual bool IsSkip(const RoboStatus &refStatus) = 0;
};

class Pipeline
{
public:
    template <typename StatusType>
    Pipeline(StatusType &status)
        : m_Status(status)
    {
        // Validate template params
        static_assert(std::is_base_of<BaseStatus, StatusType>::value,
                      "StatusType should derived from BaseStatus");
    }

    template <typename InputType>
    void ProcessTask(const InputType &refInput)
    {
        static_assert(std::is_base_of<BaseInputData, InputType>::value,
                      "InputType should derived from BaseInputData");
        static_assert(std::is_copy_constructible<InputType>::value,
                      "InputType should be copy constructible");
        Process(refInput);
    }

    template <typename ActionType>
    void RegisterAction()
    {
        static_assert(std::is_base_of<IExecutable, ActionType>::value,
                      "ActionType should derived from IExecutable");
        m_Actions.emplace_back(new ActionType);
    }

private:
    template <typename InputType, typename StatusType>
    void Process(const InputType &refInput,
                 const StatusType refStatus)
    {
        OutputType output;
        InputType input(input);
        for (auto &&action : m_Actions)
        {
            if (action.get()->IsSkip(refStatus))
            {
                action.get()->Process(input, output);
            }
        }
        return output;
    }

    std::vector<std::shared_ptr<IExecutable>> m_Actions;

    BaseStatus &m_Status;
};

} // namespace NautilusVision

#endif