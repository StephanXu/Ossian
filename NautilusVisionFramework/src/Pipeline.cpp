
#include "nv/Pipeline.hpp"

#include <vector>

namespace NautilusVision
{
namespace IOAP
{
Pipeline::Pipeline(BaseStatus &status, std::vector<IExecutable *> &&actions)
    : m_Actions(std::move(actions)),
      m_Status(status)
{
}

void Pipeline::ProcessTask(std::shared_ptr<BaseInputData> inputData)
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
} // namespace IOAP
} // namespace NautilusVision