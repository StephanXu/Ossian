/**
 * @file MultiThread.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 提供多线程支持
 * @version 0.1
 * @date 2019-10-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef MULTITHREAD_HPP
#define MULTITHREAD_HPP

#include <queue>
#include <mutex>
#include <memory>
#include <atomic>
#include <vector>
#include <future>
#include <functional>

namespace NautilusVision
{
/**
 * @brief 线程安全的队列
 * 该类提供一种多线程安全的队列实现
 * @tparam T 队列元素类型
 */
template <typename T>
class SafeQueue
{
public:
    SafeQueue() = default;

    /**
     * @brief 添加元素到队列
     * 添加一项元素到队列中
     * @param value 待添加的元素
     */
    void Push(T value)
    {
        std::shared_ptr<T> data{std::make_shared<T>(std::move(value))};
        {
            std::lock_guard<std::mutex> lock{m_Mutex};
            m_Queue.push(data);
            m_DataCondition.notify_one();
        }
    }

    /**
     * @brief 队列是否为空
     * 检查队列是否为空
     * @return true 为空
     * @return false 不为空
     */
    bool Empty() const
    {
        std::lock_guard<std::mutex> lock{m_Mutex};
        return m_Queue.empty();
    }

    size_t Size() const
    {
        std::lock_guard<std::mutex> lock{m_Mutex};
        return m_Queue.size();
    }

    /**
     * @brief 等待元素并返回
     * 当队列中有元素时，从队列中取回并弹出元素，没有元素时等待新元素出现
     * @param outValue 弹出的元素
     */
    void WaitAndPop(T &outValue)
    {
        std::unique_lock<std::mutex> lock{m_Mutex};
        m_DataCondition.wait(lock, [this]() { return !m_Queue.empty(); });
        outValue = std::move(m_Queue.front());
        m_Queue.pop();
    }

    /**
     * @brief 等待元素并返回
     * 当队列中有元素时，从队列中取回并弹出元素，没有元素时等待新元素出现
     * @return std::shared_ptr<T> 弹出元素的指针
     */
    std::shared_ptr<T> WaitAndPop()
    {
        std::unique_lock<std::mutex> lock{m_Mutex};
        m_DataCondition.wait(lock, [this]() { return !m_Queue.empty(); });
        std::shared_ptr<T> result{m_Queue.front()};
        m_Queue.pop();
        return result;
    }

    /**
     * @brief 尝试从队列中弹出元素
     * 尝试从队列中弹出元素存储到 outValue 中
     * @param outValue 弹出的元素
     * @return true 弹出， outValue 被修改
     * @return false 队列为空， outValue 不被修改
     */
    bool TryPop(T &outValue)
    {
        std::lock_guard<std::mutex> lock{m_Mutex};
        if (m_Queue.empty())
        {
            return false;
        }
        outValue = std::move(*m_Queue.front());
        m_Queue.pop();
        return true;
    }

    /**
     * @brief 尝试从队列中弹出元素
     * 尝试从队列中弹出元素并返回其指针
     * @return std::shared_ptr<T> 弹出元素的指针
     */
    std::shared_ptr<T> TryPop()
    {
        std::lock_guard<std::mutex> lock{m_Mutex};
        if (m_Queue.empty())
        {
            return std::shared_ptr<T>();
        }
        std::shared_ptr<T> result{m_Queue.front()};
        m_Queue.pop();
        return result;
    }

private:
    std::queue<std::shared_ptr<T>> m_Queue;
    mutable std::mutex m_Mutex;
    std::condition_variable m_DataCondition;
};

/**
 * @brief 任务包装器
 * 此任务包装器可以包装调用 void() 类型的函数（或std::bind, std::packaged_task等）
 * 包装器不允许复制，只可以创建和移动。
 */
class TaskWrapper
{
public:
    TaskWrapper() = default;
    template <typename F>
    TaskWrapper(F &&f) : m_Impl(new ImplType<F>(std::move(f))) {}
    TaskWrapper(TaskWrapper &&t) noexcept : m_Impl(std::move(t.m_Impl)) {}
    TaskWrapper(const TaskWrapper &t) = delete;
    TaskWrapper &operator=(TaskWrapper &&t) noexcept
    {
        m_Impl = std::move(t.m_Impl);
        return *this;
    }
    TaskWrapper &operator=(const TaskWrapper &t) = delete;

    void operator()()
    {
        m_Impl->Impl();
    }

private:
    /**
     * @brief 执行体基类
     * 利用此基类以调用不同类型的函数
     */
    class ImplBase
    {
    public:
        virtual void Impl() = 0;
        virtual ~ImplBase(){};
    };
    /**
     * @brief 执行体子类
     * 在 TaskWrapper 的构造函数中完成实例化，以表现不同类型的函数
     * @tparam F 函数（或其他的调用）类型
     */
    template <typename F>
    class ImplType : public ImplBase
    {
    public:
        F m_F;
        ImplType(F &&f) : m_F(std::move(f)) {}
        void Impl() override { m_F(); }
    };

    std::unique_ptr<ImplBase> m_Impl;
};

/**
 * @brief 线程队列保护
 * 此类型接受一个线程容器（形如 std::vector 定义方式的类型）。
 * 当此类型析构时，将逐一执行容器中元素的 join() 方法。
 * [注意]：当该类作为成员时，应当在线程列表的下方声明。
 * @tparam ContainerType 容器类型
 */
template <typename ContainerType = std::vector<std::thread>>
class ThreadsGuard
{
public:
    explicit ThreadsGuard(ContainerType &threads)
        : m_Threads(threads)
    {
    }

    ~ThreadsGuard()
    {
        for (auto &&item : m_Threads)
        {
            if (item.joinable())
            {
                item.join();
            }
        }
    }

private:
    ContainerType &m_Threads;
};

/**
 * @brief 线程池
 * 提供一个可接受返回结果的线程池
 */
class ThreadPool
{
public:
    /**
     * @brief 初始化线程池
     * 初始化一个线程池
     * @param threadNum 线程数目
     */
    ThreadPool(const int threadNum)
        : m_TerminateSignal(false), m_Guard(m_Threads)
    {
        try
        {
            for (int i{}; i < threadNum; ++i)
            {
                m_Threads.emplace_back(&ThreadPool::Worker, this);
            }
        }
        catch (...)
        {
            m_TerminateSignal = true;
            throw;
        }
    }
    /**
     * @brief 初始化线程池
     * 使用自动判断的线程数初始化线程池
     */
    ThreadPool() : ThreadPool(std::thread::hardware_concurrency()) {}
    ~ThreadPool()
    {
        m_TerminateSignal = true;
    }

    /**
     * @brief 添加一个任务
     * 向任务队列中添加一个任务
     * @tparam F 任务函数类型
     * @tparam Args 任务函数的参数类型
     * @param f 任务函数
     * @param args 任务函数的参数
     * @return std::future<typename std::result_of<F(Args...)>::type> 返回值接收
     */
    template <typename F, typename... Args>
    std::future<typename std::result_of<F(Args...)>::type> AddTask(F f, Args &&... args)
    {
        typedef typename std::result_of<F(Args...)>::type ResultType;
        auto binded = std::bind(f, std::forward<Args>(args)...);
        std::packaged_task<ResultType()> task{std::move(binded)};
        auto fut = task.get_future();
        m_Tasks.Push(std::move(task));
        return fut;
    }

    bool Empty() const
    {
        return m_Tasks.Empty();
    }

	size_t WaitingCount() const
	{
		return m_Tasks.Size();
	}

private:
    SafeQueue<TaskWrapper> m_Tasks;
    std::atomic_bool m_TerminateSignal; //< 要结束时为true
    std::vector<std::thread> m_Threads;
    ThreadsGuard<std::vector<std::thread>> m_Guard; //< 保护析构时线程被join

    /**
     * @brief 工作函数
     * 任务在此函数中被调用
     */
    void Worker()
    {
        while (!m_TerminateSignal)
        {
            TaskWrapper task;
            if (m_Tasks.TryPop(task))
            {
                task();
            }
            else
            {
                std::this_thread::yield();
            }
        }
    }
};

} // namespace NautilusVision

#endif