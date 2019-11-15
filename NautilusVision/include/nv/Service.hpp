﻿/**
 * @file Service.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 处理服务相关实现
 * @version 0.1
 * @date 2019-10-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef SERVICE_HPP
#define SERVICE_HPP

#include "IOTypes.hpp"

#include <memory>
#include <string>
#include <future>
#include <typeindex>

namespace NautilusVision
{
namespace IOAP
{
/**
 * @brief 服务对象接口
 */
class IService
{
public:
private:
};

/**
 * @brief Create a Service object
 * 创建一个服务对象
 * @tparam ServiceType 服务类型
 * @return std::unique_ptr<ServiceType> 服务对象实例
 */
template <typename ServiceType>
std::unique_ptr<ServiceType> CreateService()
{
    static_assert(std::is_base_of<IService, ServiceType>::value, "ServiceType should derived from IService");
    return std::make_unique<ServiceType>();
}

/**
 * @brief 输入方式基类
 * 数据 BaseInputData 通过该类获得，并可在 Dispatcher 中传递给相匹配的 Pipeline
 */
class BaseInputAdapter : IService
{
public:
    /**
     * @brief 获得输入
     * 以同步方式获得输入，可能引起线程阻塞以等待输入
     * @return std::shared_ptr<InputDataType> 输入数据的指针
     */
    virtual std::shared_ptr<BaseInputData> GetInput() = 0;

    /**
     * @brief 获得输入
     * 以异步方式获得输入
     * @return std::future<std::shared_ptr<InputDataType>> 输入数据指针的 future
     */
    virtual std::future<std::shared_ptr<BaseInputData>> GetInputAsync() = 0;

    /**
     * @brief 获得输入类型索引关键字
     * 获得该输入方式所获得的数据类型索引关键字
     * @return std::type_index 关键字
     */
    virtual std::type_index GetInputTypeIndex() const = 0;
};
} // namespace IOAP
} // namespace NautilusVision

#endif