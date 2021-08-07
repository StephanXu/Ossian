#ifndef OSSIAN_CORE_FACTORY
#define OSSIAN_CORE_FACTORY

#include <functional>
#include <memory>

namespace ossian
{

/**
 * @brief 将构造函数声明为工厂函数
 * 
 * 利用该宏将自动为构造函数添加签名信息用于依赖注入。例如：
 *
 * class MyService
 * {
 * public:
 *     OSSIAN_SERVICE_SETUP(MyService(Foo* foo, Bar* bar)) {...}
 * };
 *
 * 其等效于：
 * 
 * class MyService
 * {
 * public:
 *     using _OssianServiceInjectorType = MyService(Foo* foo, Bar* bar);
 *     MyService(Foo* foo, Bar* bar) {...}
 * };
 *
 * 注意：
 * - 该宏所包含的构造函数参数均应是native指针。
 * - 不支持使用模板构造函数（但可使用模板类）
 * - 不支持函数声明中包含默认参数
 * - 不支持函数声明中带有explicit
 * 如果无法避免地不能满足上述要求，请自行定义 _OssianServiceInjectorType 或自行建立工厂
 * 函数。
 */
#define OSSIAN_SERVICE_SETUP(Signature)	\
	using _OssianServiceInjectorType=Signature;	\
	Signature

template<typename ServiceType>
struct InstanceFactory;

template<typename ServiceType, typename ...Deps>
struct InstanceFactory<ServiceType(Deps...)>
{
    static auto Value(std::function<void(ServiceType&)> configProc)
    {
        return std::function<std::unique_ptr<ServiceType>(Deps...)>
        {
            [configProc](Deps...deps)
            {
                auto instance = std::make_unique<ServiceType>(deps...);
                if (configProc)
                {
                    configProc(*instance);
                }
                return std::move(instance);
            }
        };
    }
};

} // ossian

#endif // OSSIAN_CORE_FACTORY