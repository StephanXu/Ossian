
#ifndef PIPLINE_HPP
#define PIPLINE_HPP

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
    virtual void Process(const BaseInput &refInput,
                         BaseOutput &outResult) = 0;
    virtual bool IsSkip(const RoboStatus &refStatus) = 0;
};

template <typename InputType,
          typename OutputType,
          typename StatusType,
          size_t threadCapacity>
class BasePipline
{
public:
    explicit BasePipline()
    {
        // Validate template params
        static_assert(std::is_base_of<BaseInput, InputType>::value,
                      "InputType should derived from BaseInput");
        static_assert(std::is_base_of<BaseOutput, OutputType>::value,
                      "OutputType should derived from BaseOutput");
        static_assert(std::is_base_of<BaseStatus, StatusType>::value,
                      "StatusType should derived from BaseStatus");
    }

    OutputType ProcessTask(const InputType &refInput)
    {
        return Process(refInput);
    }

    std::future<OutputType> ProcessTaskAsync(const InputType &refInput)
    {
        std::packaged_task<OutputType(refInput)> task{Process};
        std::thread t{std::move(Process), this, refInput};
        t.detach();
        return task.get_future();
    }

    template <typename ActionType>
    void RegisterAction()
    {
        static_assert(std::is_base_of<IExecutable, ActionType>::value,
                     "ActionType should derived from IExecutable");
        m_Actions.emplace_back(new ActionType);
    }

private:
    OutputType Process(const InputType &refInput,
                       const StatusType refStatus)
    {
        OutputType output;
        InputType input;
        refInput.CopyTo(input);
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

    StatusType m_Status;

    enum class PiplineStatus
    {
        AVAILABLE
    };
};

} // namespace NautilusVision

#endif