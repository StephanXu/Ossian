﻿/**
 * @file ossian.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 包含所有Ossian Core的头文件
 * @version 0.2
 * @date 2020-1-29
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_CORE_OSSIAN
#define OSSIAN_CORE_OSSIAN

#include "ossian/DI.hpp"
#include "ossian/Pipeline.hpp"
#include "ossian/Dispatcher.hpp"
#include "ossian/MultiThread.hpp"
#include "ossian/Service.hpp"
#include "ossian/ApplicationBuilder.hpp"
#include "ossian/Factory.hpp"
#include "ossian/Configuration.hpp"
#include "ossian/Motor.hpp"

// IO
#include "ossian/IOListener.hpp"
#include "ossian/io/IO.hpp"
#include "ossian/io/SerialPort.hpp"
#include "ossian/io/CAN.hpp"
#include "ossian/io/UART.hpp"
#include "ossian/io/IOError.hpp"

#endif // OSSIAN_CORE_OSSIAN