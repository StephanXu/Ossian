﻿#ifndef OSSIAN_CORE_IO_BUFFERPOOL
#define  OSSIAN_CORE_IO_BUFFERPOOL
#include "ossian/MultiThread.hpp"
#include "boost/circular_buffer.hpp"

#include <mutex>
namespace ossian
{
/**
 * @brief 线程池
 * 提供一个可接受返回结果的线程池
 */
class BufferPool
{
public:
    /**
     * @brief 初始化线程池
     * 初始化一个线程池
     * @param threadNum 线程数目
     */
    BufferPool(const int size)
        :m_Tasks(size)
    {
    }
    ~BufferPool() = default;

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
    std::future<typename std::result_of<F(Args...)>::type> AddTask(F f, Args&&... args)
    {
        std::lock_guard<std::mutex> lock{ m_Mutex };
        typedef typename std::result_of<F(Args...)>::type ResultType;
        auto binded = std::bind(f, std::forward<Args>(args)...);
        std::packaged_task<ResultType()> task{ std::move(binded) };
        auto fut = task.get_future();
        m_Tasks.push_back(std::move(task));
        return fut;
    }

    bool Empty() const
    {
        std::lock_guard<std::mutex> lock{ m_Mutex };
        return m_Tasks.empty();
    }

    size_t WaitingCount() const
    {
        std::lock_guard<std::mutex> lock{ m_Mutex };
        return m_Tasks.size();
    }

    // 尝试取出数据并处理
    void Process()
    {
        while (!Empty())
        {
            TaskWrapper task;
            task = std::move(m_Tasks.front());
            m_Tasks.pop_front();
            task();
        }
    }

private:
    mutable std::mutex m_Mutex;
    boost::circular_buffer<TaskWrapper> m_Tasks;
};
} // ossian
#endif // OSSIAN_CORE_IO_BUFFERPOOL