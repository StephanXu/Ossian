#ifndef FACTORY_HPP
#define FACTORY_HPP

#include "Service.hpp"

namespace NautilusVision
{

class Factory
{
public:
    /**
     * @brief Create a Service object
     * 创建一个带有配置依赖的一般服务对象
     * @tparam ServiceType 服务类型
     * @return std::unique_ptr<ServiceType> 服务对象实例
     */
    template <typename ServiceType>
    static std::unique_ptr<ServiceType> CreateGeneralService(Utils::ConfigLoader* config)
    {
        static_assert(std::is_base_of<IOAP::IService, ServiceType>::value, "ServiceType should derived from IService");
        return std::unique_ptr<ServiceType>(new ServiceType(config));
    }


    /**
     * @brief Create a Service object
     * 创建一个不带有配置依赖的一般服务对象
     * @tparam ServiceType 服务类型
     * @return std::unique_ptr<ServiceType> 服务对象实例
     */
    template <typename ServiceType>
    static std::unique_ptr<ServiceType> CreateGeneralServiceWithoutConfig()
    {
        static_assert(std::is_base_of<IOAP::IService, ServiceType>::value, "ServiceType should derived from IService");
        return std::unique_ptr<ServiceType>();
    }

    /**
     * @fn	template<typename T> std::unique_ptr<T> CreateGeneralObject(Utils::Configuration* config)
     *
     * @brief	用于仅接收一个配置依赖项的类型的通用工厂函数
     *
     * @tparam	T	Generic type parameter.
     * @param [in]	config	配置依赖项.
     *
     * @returns	创建的对象
     */
    template<typename T>
    static std::unique_ptr<T> CreateGeneralObject(Utils::ConfigLoader* config)
    {
        return std::unique_ptr<T>(new T(config));
    }
};

} // NautilusVision

#endif // FACTORY_HPP