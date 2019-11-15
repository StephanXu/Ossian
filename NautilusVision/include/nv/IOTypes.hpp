/**
 * @file IOTypes.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 输入输入类型
 * @version 0.1
 * @date 2019-10-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef IOTYPES_HPP
#define IOTYPES_HPP

#include <future>
#include <memory>
#include <string>
#include <typeindex>

namespace NautilusVision
{
namespace IOAP
{
/**
 * @brief 数据数据基类
 */
class BaseInputData
{
public:
    virtual std::unique_ptr<BaseInputData> Clone() const = 0;
    virtual BaseInputData *GetData() = 0;
    virtual std::type_index GetTypeIndex() const
    {
        return std::type_index(typeid(*this));
    }
};

/**
 * @brief 输出数据基类
 */
class BaseOutputData
{
};

/**
 * @brief 状态基类
 */
class BaseStatus
{
};

/**
 * @brief Create a Status object
 * 实例化状态类
 * @tparam StatusType 状态类型
 * @return std::unique_ptr<StatusType> 状态实例
 */
template <typename StatusType>
std::unique_ptr<StatusType> CreateStatus()
{
    static_assert(std::is_base_of<BaseStatus, StatusType>::value, "StatusType should derived from BaseStatus");
    return std::make_unique<StatusType>();
}
} // namespace IOAP
} // namespace NautilusVision

#endif