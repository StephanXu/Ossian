/**
 * @file ossian.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 包含所有NautilusVisionFramework的头文件
 * @version 0.1
 * @date 2019-10-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef OSSIAN_CORE_OSSIAN
#define OSSIAN_CORE_OSSIAN

#include "ossian/IOTypes.hpp"
#include "ossian/DI.hpp"
#include "ossian/Pipeline.hpp"
#include "ossian/Dispatcher.hpp"
#include "ossian/MultiThread.hpp"
#include "ossian/Service.hpp"
#include "ossian/Config.hpp"
#include "ossian/Factory.hpp"
#include "ossian/Configuration.hpp"

// IO
#include "ossian/io/SerialPort.hpp"
#include "ossian/io/CAN.hpp"

#endif // OSSIAN_CORE_OSSIAN