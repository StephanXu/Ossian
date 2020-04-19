#include "ossian/MultiThread.hpp"
#include "boost/circular_buffer.hpp"
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
    {
        m_Tasks = std::make_unique<boost::circular_buffer<TaskWrapper>>(size);
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
        typedef typename std::result_of<F(Args...)>::type ResultType;
        auto binded = std::bind(f, std::forward<Args>(args)...);
        std::packaged_task<ResultType()> task{ std::move(binded) };
        auto fut = task.get_future();
        m_Tasks->push_back(std::move(task));
        return fut;
    }

    bool Empty() const
    {
        return m_Tasks->empty();
    }

    size_t WaitingCount() const
    {
        return m_Tasks->size();
    }

    // 尝试取出数据并处理
    void Process()
    {
        while (!Empty())
        {
            TaskWrapper task;
            task = m_Tasks->pop_front();
            task();
        }
    }

private:
    std::unique_ptr<boost::circular_buffer<TaskWrapper>> m_Tasks;
};
}