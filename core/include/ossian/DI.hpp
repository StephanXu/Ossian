/**
 * @file DI.hpp
 * @brief 此文件提供依赖注入相关实现
 */
#ifndef OSSIAN_CORE_DI
#define OSSIAN_CORE_DI

#include <spdlog/spdlog.h>

#include <memory>
#include <atomic>
#include <unordered_map>
#include <functional>
#include <vector>
#include <typeinfo>
#include <typeindex>
#include <string>
#include <algorithm>

namespace ossian
{
namespace DI
{
class DuplicateRegistration : std::exception
{
};

//------------------------------------------------------------------------------
// 类型封装

/**
 * @brief 建立由 ValueType 包装的类型映射
 * 此类型维护一个由类型的hash作为key的hash表
 * @tparam ValueType 包装器类型
 */
template <class ValueType>
class TypeMap
{
	using Container = std::unordered_map<std::type_index, ValueType>;

public:
	using iterator = typename Container::iterator;
	using const_iterator = typename Container::const_iterator;

	iterator begin() { return m_Container.begin(); }
	iterator end() { return m_Container.end(); }
	const_iterator begin() const { return m_Container.begin(); }
	const_iterator end() const { return m_Container.end(); }
	const_iterator cbegin() const { return m_Container.cbegin(); }
	const_iterator cend() const { return m_Container.cend(); }

	template <class Key>
	iterator find() { return m_Container.find(std::type_index(typeid(Key))); }

	iterator find(std::type_index typeIndex) { return m_Container.find(typeIndex); }

	template <class Key>
	const_iterator find() const { return m_Container.find(std::type_index(typeid(Key))); }

	const_iterator find(std::type_index typeIndex) const { return m_Container.find(typeIndex); }

	template <class Key>
	void put(ValueType&& value)
	{
		m_Container[std::type_index(typeid(Key))] = std::forward<ValueType>(value);
	}

private:
	Container m_Container;
};

/**
 * @brief 抽象的实例容器
 */
class AbstractInstanceContainer
{
public:
	virtual ~AbstractInstanceContainer() = default;
	virtual void* Get() = 0;
};

/**
 * @brief 实例容器
 * 在该示例容器中维护一个指针保存实例地址
 * @tparam T 实例类型
 * @tparam Deleter 实例deleter（被使用于 std::unique_ptr）
 */
template <class T, class Deleter>
class InstanceContainer : public AbstractInstanceContainer
{
public:
	explicit InstanceContainer(std::unique_ptr<T, Deleter>&& p)
		: m_Pointer(std::move(p))
	{
	}

	void* Get() { return static_cast<void*>(m_Pointer.get()); }

private:
	std::unique_ptr<T, Deleter> m_Pointer;
};

/**
 * @brief 包装一个实例到实例容器中
 * @tparam T 实例类型
 * @tparam Deleter 实例deleter（被使用于 std::unique_ptr）
 * @param ptr 实例指针
 * @return std::unique_ptr<AbstractInstanceContainer> 创建的实例容器
 */
template <class T, class Deleter>
std::unique_ptr<AbstractInstanceContainer> WrapIntoInsatanceContainer(std::unique_ptr<T, Deleter>&& ptr)
{
	return std::make_unique<InstanceContainer<T, Deleter>>(std::move(ptr));
}

class DIConfiguration;

/**
 * @brief Collection服务。
 * 每个接口将同时生成一个Collection服务，用于接口的多实现
 * @tparam T
 */
template <class T>
class ServiceCollection
{
	using iterator = typename std::vector<T*>::iterator;
	using const_iterator = typename std::vector<T*>::const_iterator;
	using reverse_iterator = typename std::vector<T*>::reverse_iterator;
	using const_reverse_iterator = typename std::vector<T*>::const_reverse_iterator;
	friend class DIConfiguration;

	ServiceCollection(std::vector<T*> servicesTypeIndex) : m_Services(servicesTypeIndex)
	{
	}

	void AddServiceInstance(void* servicePointer)
	{
		m_Services.push_back(static_cast<T*>(servicePointer));
	}

	T* GetLastOne()
	{
		return *m_Services.rbegin();
	}

	std::vector<T*> m_Services;

public:
	iterator begin() { return m_Services.begin(); }
	iterator end() { return m_Services.end(); }

	const_iterator begin() const { return m_Services.begin(); }
	const_iterator end() const { return m_Services.end(); }

	const_iterator cbegin() const { return m_Services.cbegin(); }
	const_iterator cend() const { return m_Services.cend(); }

	reverse_iterator rbegin() { return m_Services.rbegin(); }
	reverse_iterator rend() { return m_Services.rend(); }

	const_reverse_iterator rbegin() const { return m_Services.rbegin(); }
	const_reverse_iterator rend() const { return m_Services.rend(); }

	const_reverse_iterator crbegin() const { return m_Services.crbegin(); }
	const_reverse_iterator crend() const { return m_Services.crend(); }

	size_t size() const { return m_Services.size(); }
};

//------------------------------------------------------------------------------
// 注入器

/**
 * @brief 工厂函数类型
 * @tparam InstanceType 实例类型
 * @tparam Deleter 实例deleter（被使用于 std::unique_ptr）
 * @tparam Deps 实例的依赖
 */
template <class InstanceType, class Deleter, class... Deps>
using InstanceFactoryFunction = std::function<std::unique_ptr<InstanceType, Deleter>(Deps*...)>;

template <class InstanceType, class Deleter, class... Deps>
using InstanceFactoryNativeFunction = std::unique_ptr<InstanceType, Deleter>(*)(Deps*...);

/**
 * @brief 注入器
 * 此类型维护实例列表，分发实例。
 * 此类不允许直接构造，且不能够复制。应当使用 DIConfiguration::BuildInjector 函数构造
 */
class Injector
{
	friend class DIConfiguration;

public:
	Injector(Injector&& other) noexcept { *this = std::move(other); }

	Injector& operator=(Injector&& other) noexcept
	{
		m_InstanceMap = std::move(other.m_InstanceMap);
		return *this;
	}

	template <class T, class Dependee = std::nullptr_t>
	T* GetInstance() const
	{
		auto it = m_InstanceMap.find<ServiceCollection<T>>();
		if (it != m_InstanceMap.end())
		{
			return *(static_cast<ServiceCollection<T>*>(it->second->Get())->rbegin());
		}

		it = m_InstanceMap.find<T>();
		if (it == m_InstanceMap.end())
		{
			throw std::runtime_error(std::string(typeid(T).name()) +
			                         ": unsatisfied dependency of " +
			                         std::string(typeid(Dependee).name()));
		}
		return static_cast<T*>(it->second->Get());
	}

private:
	template <class InstanceType, class Deleter, class... Deps>
	std::unique_ptr<InstanceType, Deleter>
	Inject(InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory) const
	{
		return instanceFactory(GetInstance<typename std::remove_const_t<Deps>,
		                                   typename std::remove_const_t<InstanceType>>()...);
	}

	void* GetInstance(std::type_index typeIndex)
	{
		auto it = m_InstanceMap.find(typeIndex);
		if (it == m_InstanceMap.end())
		{
			throw std::runtime_error(std::string(typeIndex.name()) +
			                         ": unsatisfied dependency");
		}
		return it->second->Get();
	}

	Injector() = default;

	using InstanceMap = TypeMap<std::unique_ptr<AbstractInstanceContainer>>;
	InstanceMap m_InstanceMap;
};

/**
 * @brief 依赖注入配置类
 * 在此类中完成依赖项注册等内容
 */
class DIConfiguration
{
public:
	/**
	 * @brief 注册服务/依赖
	 * 通过该函数注册一项服务/依赖，服务/依赖需要由一个工厂函数提供创建方法
	 * @tparam InstanceType 实例类型
	 * @tparam Deleter std::unique_ptr 将使用的deleter
	 * @tparam Deps 依赖项
	 * @param instanceFactory 工厂函数
	 */
	template <class InterfaceType, class InstanceType, class Deleter, class... Deps>
	void Add(InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory)
	{
		std::type_index instanceTypeId = std::type_index(typeid(typename std::remove_const_t<InstanceType>));
		DependencyNode& node           = m_Graph[instanceTypeId];
		node.m_DebugTypeName           = typeid(typename std::remove_const_t<InterfaceType>).name();
		node.m_Initializer             = [instanceFactory](Injector& inj)
		{
			// 此处无法支持同一个服务注册到两个接口
			auto instance = WrapIntoInsatanceContainer(inj.Inject(instanceFactory));
			inj.m_InstanceMap.put<InstanceType>(std::move(instance));
		};
		node.m_HasInitializer = true;
		node.m_DebugTypeName  = typeid(typename std::remove_const_t<InstanceType>).name();
		node.m_Dependencies   = {
			std::type_index(typeid(typename std::remove_const_t<Deps>))...,
			std::type_index(typeid(std::remove_const_t<ServiceCollection<Deps>>))...
		};

		std::type_index interfaceTypeId =
			std::type_index(typeid(typename std::remove_const_t<ServiceCollection<InterfaceType>>));
		DependencyNode& collectionNode = m_Graph[interfaceTypeId];
		collectionNode.m_Dependencies.push_back(instanceTypeId);
		collectionNode.m_Initializer = [dependencies = collectionNode.m_Dependencies](Injector& inj)
		{
			std::vector<InterfaceType*> services;
			for (auto&& item : dependencies)
			{
				services.push_back(static_cast<InterfaceType*>(inj.GetInstance(item)));
			}
			auto instance = WrapIntoInsatanceContainer(
				std::unique_ptr<ServiceCollection<InterfaceType>>(new ServiceCollection<InterfaceType>(services)));
			inj.m_InstanceMap.put<ServiceCollection<InterfaceType>>(std::move(instance));
		};
		collectionNode.m_HasInitializer = true;
	}

	template <class InterfaceType, class InstanceType, class Deleter, class... Deps>
	void Add(InstanceFactoryNativeFunction<InstanceType, Deleter, Deps...> instanceFactory)
	{
		InstanceFactoryFunction<InstanceType, Deleter, Deps...> factoryFunction{instanceFactory};
		Add<InterfaceType>(factoryFunction);
	}

	/**
	 * @brief 建立注入器
	 * 该方法应该在服务（依赖）注册完毕后调用
	 * @return Injector 注入器
	 */
	Injector BuildInjector()
	{
		Injector injector;

		for (auto& node : m_Graph)
		{
			// This test is logically redundant, it's just for better performance.
			if (node.second.m_Mark == DependencyNode::Mark::Unmarked)
			{
				ToposortVisitNode(node.first, injector);
			}
		}
		return injector;
	}

private:
	using InitializerFunction = std::function<void(Injector&)>;

	/**
	 * @brief 依赖图节点
	 */
	class DependencyNode
	{
	public:
		enum class Mark
		{
			Unmarked,
			Temp,
			Marked
		};

		Mark m_Mark = Mark::Unmarked;

		std::string m_DebugTypeName;       //< 依赖类型（调试用途）
		InitializerFunction m_Initializer; //< 依赖建造函数，在此函数中将调用工厂函数并添加到依赖容器
		bool m_HasInitializer = false;     //< 如果为false，则说明节点被依赖但没有提供工厂（即没有注册），这将引起异常
		std::vector<std::type_index> m_Dependencies;
	};

	/**
	 * @brief 实例化依赖
	 * 通过对依赖图拓扑遍历依次实例化依赖
	 * @param nodeId 节点关键字
	 * @param injector 注入器
	 */
	void ToposortVisitNode(std::type_index nodeId, Injector& injector)
	{
		DependencyNode& node = m_Graph[nodeId];
		if (node.m_Mark == DependencyNode::Mark::Temp)
		{
			throw std::runtime_error(node.m_DebugTypeName + " appears to be part of a cycle");
		}
		else if (node.m_Mark == DependencyNode::Mark::Unmarked)
		{
			node.m_Mark = DependencyNode::Mark::Temp;
			for (std::type_index dependent : node.m_Dependencies)
			{
				// If it's a fake dependent, ignore it.
				if (m_Graph.find(dependent) != m_Graph.end())
				{
					ToposortVisitNode(dependent, injector);
				}
			}
			node.m_Mark = DependencyNode::Mark::Marked;
			if (node.m_HasInitializer)
			{
				node.m_Initializer(injector);
			}
		}
	}

	std::unordered_map<std::type_index, DependencyNode> m_Graph;
};
} // namespace DI
} // namespace ossian

#endif //OSSIAN_CORE_DI
