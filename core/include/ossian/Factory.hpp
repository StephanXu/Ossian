#ifndef OSSIAN_CORE_FACTORY
#define OSSIAN_CORE_FACTORY

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

template<typename T>
struct InstanceFactory;

template<typename R, typename ...Deps>
struct InstanceFactory<R(Deps...)>
{
	static auto Value(std::function<void(R&)> configProc) -> std::function<std::unique_ptr<R>(Deps...)>
	{
		return std::function<std::unique_ptr<R>(Deps...)>
		{
			[configProc](Deps...deps)
			{
				auto instance = std::make_unique<R>(deps...);
				if (configProc)
				{
					configProc(*instance);
				}
				return instance;
			}
		};
	}
};

//class Factory
//{
//public:
//	template<class ServiceType, class ...Deps>
//	static std::unique_ptr<ServiceType> CreateGeneralService(Deps*...deps)
//	{
//		return std::unique_ptr<ServiceType>(new ServiceType(deps...)); ///< std::make_unique无法访问ServiceType的私有域
//	}
//
//	template<class ConfigureService, class ConfigureProc>
//	static std::unique_ptr<ConfigureService> CreateGeneralServiceConfigure(ConfigureProc configureProc)
//	{
//		auto instance = std::unique_ptr<ConfigureService>(new ConfigureService);
//		configureProc(*instance);
//		return instance;
//	}
//};

} // ossian

#endif // OSSIAN_CORE_FACTORY