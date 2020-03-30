/**
 * @file Pipeline.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 处理管道相关支持
 * @version 0.1
 * @date 2019-10-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef OSSIAN_CORE_PIPELINE
#define OSSIAN_CORE_PIPELINE

#include <vector>

namespace ossian
{

/**
 * @brief 执行体
 * 执行体定义了可执行的实例，可以被执行，并且提供一个可跳过的判断方法
 */
class IExecutable
{
public:
	virtual void ExecuteProc() = 0;
};

} // namespace ossian

#endif //OSSIAN_CORE_PIPELINE
