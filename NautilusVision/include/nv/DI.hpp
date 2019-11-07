/**
 * @file DI.hpp
 * @brief 此文件提供依赖注入相关实现
 */
#ifndef DI_HPP
#define DI_HPP

#include <memory>
#include <atomic>
#include <unordered_map>
#include <functional>
#include <vector>
#include <typeinfo>
#include <typeindex>
#include <string>

namespace NautilusVision
{
namespace DI
{

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

    template <class Key>
    const_iterator find() const { return m_Container.find(std::type_index(typeid(Key))); }

    template <class Key>
    void put(ValueType &&value)
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
    virtual void *Get() = 0;
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
    explicit InstanceContainer(std::unique_ptr<T, Deleter> &&p)
        : m_Pointer(std::move(p))
    {
    }

    void *Get() { return static_cast<void *>(m_Pointer.get()); }

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
std::unique_ptr<AbstractInstanceContainer> WrapIntoInsatanceContainer(std::unique_ptr<T, Deleter> &&ptr)
{
    return std::make_unique<InstanceContainer<T, Deleter>>(std::move(ptr));
}

//------------------------------------------------------------------------------
// 注入器

/**
 * @brief 工厂函数类型
 * @tparam InstanceType 实例类型
 * @tparam Deleter 实例deleter（被使用于 std::unique_ptr）
 * @tparam Deps 实例的依赖
 */
template <class InstanceType, class Deleter, class... Deps>
using InstanceFactoryFunction = std::unique_ptr<InstanceType, Deleter> (*)(Deps *...);

/**
 * @brief 注入器
 * 此类型维护实例列表，分发实例。
 * 此类不允许直接构造，且不能够复制。应当使用 DIConfiguration::BuildInjector 函数构造
 */
class Injector
{
    friend class DIConfiguration;

public:
    Injector(Injector &&other) noexcept { *this = std::move(other); }
    Injector &operator=(Injector &&other) noexcept
    {
        m_InstanceMap = std::move(other.m_InstanceMap);
        return *this;
    }

    template <class T, class Dependee = std::nullptr_t>
    T *GetInstance() const
    {
        auto it = m_InstanceMap.find<T>();
        if (it == m_InstanceMap.end())
        {
            throw std::runtime_error(std::string(typeid(T).name()) +
                                     ": unsatisfied dependency of " +
                                     std::string(typeid(Dependee).name()));
        }
        return static_cast<T *>(it->second->Get());
    }

    template <class InstanceType, class Deleter, class... Deps>
    std::unique_ptr<InstanceType, Deleter>
    Inject(InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory) const
    {
        return instanceFactory(GetInstance<typename std::remove_const_t<Deps>,
                                           typename std::remove_const_t<InstanceType>>()...);
    }

private:
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
    template <class InstanceType, class Deleter, class... Deps>
    void Add(InstanceFactoryFunction<InstanceType, Deleter, Deps...> instanceFactory)
    {
        std::type_index instanceTypeId = std::type_index(typeid(typename std::remove_const_t<InstanceType>));
        DependencyNode &node = m_Graph[instanceTypeId];
        node.m_Initializer = [instanceFactory](Injector &inj) {
            auto instance = WrapIntoInsatanceContainer(inj.Inject(instanceFactory));
            inj.m_InstanceMap.put<InstanceType>(std::move(instance));
        };
        node.m_HasInitializer = true;
        node.m_DebugTypeName = typeid(typename std::remove_const_t<InstanceType>).name();
        node.m_Dependencies = {std::type_index(typeid(typename std::remove_const_t<Deps>))...};
    }

    /**
     * @brief 建立注入器
     * 该方法应该在服务（依赖）注册完毕后调用
     * @return Injector 注入器
     */
    Injector BuildInjector()
    {
        Injector injector;

        for (auto &node : m_Graph)
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
    using InitializerFunction = std::function<void(Injector &)>;
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
    void ToposortVisitNode(std::type_index nodeId, Injector &injector)
    {
        DependencyNode &node = m_Graph[nodeId];
        if (node.m_Mark == DependencyNode::Mark::Temp)
        {
            throw std::runtime_error(node.m_DebugTypeName + " appears to be part of a cycle");
        }
        else if (node.m_Mark == DependencyNode::Mark::Unmarked)
        {
            node.m_Mark = DependencyNode::Mark::Temp;
            for (std::type_index dependent : node.m_Dependencies)
            {
                ToposortVisitNode(dependent, injector);
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

} // namespace NautilusVision

#endif