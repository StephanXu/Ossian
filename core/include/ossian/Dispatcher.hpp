/**
 * @file Dispatcher.hpp
 * @author Xu Zihan (im.xuzihan@outlook.com)
 * @brief 调度器相关实现
 * @version 0.1
 * @date 2019-10-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef OSSIAN_CORE_DISPATCHER
#define OSSIAN_CORE_DISPATCHER

#include "DI.hpp"

namespace ossian
{

// 位于 Config.hpp
class ApplicationBuilder;

/**
 * @brief 分发器类型
 * 分发器类型应当由 ApplicationBuilder 创建
 */
class Dispatcher
{
    friend class ApplicationBuilder;

public:
    Dispatcher() = delete;
    ~Dispatcher() = default;
	
    Dispatcher(const Dispatcher &dispatcher) = delete;
    Dispatcher(Dispatcher&& dispatcher) = delete;

    Dispatcher& operator=(const Dispatcher& rhs) = delete;
    Dispatcher& operator=(Dispatcher&& rhs) = delete;

    void Run() const;

private:
    /**
     * @brief Construct a new Dispatcher object
     * 创建一个分发器对象
     * @param injector 实例化器
     */
    explicit Dispatcher(DI::Injector &&injector);

    DI::Injector m_Injector;
};

} // namespace ossian

#endif // OSSIAN_CORE_DISPATCHER