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
#ifndef OSSIAN_CORE_IOTYPES
#define OSSIAN_CORE_IOTYPES

#include <future>
#include <memory>
#include <string>
#include <typeindex>

namespace ossian
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
 * @brief 状态基类
 */
class BaseStatus
{
};

} // namespace IOAP
} // namespace ossian

#endif //OSSIAN_CORE_IOTYPES