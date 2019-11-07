# 快速上手

## 介绍

NV Framework(Nautilus Vision Framework)是用于机器人用计算机视觉的良好框架。框架中集成了机器人有可能用到的事务处理、数据输入输出等功能。利用本框架，你可以快速地创建出稳定安全的机器人视觉程序。

## 基本概念

NV Framework所实现的事务核心是处理输入信息。在事务处理模型中，包括状态(Status)、输入方式(Input Adapter)、输入数据(Input Data)、处理管道(Pipeline)和处理动作(Executable)。数据传递的具体方式为：

~~~
+----------------+                 +------------+
|                |    InputData    |            |
|  InputAdapter  +----------------->  Pipeline  |
|                |                 |            |
+----------------+                 +-----+------+
                                         |
       +---------------------------------+
       |
+------v------+               +-------------+
|             |               |             |
| Executable1 +--- ...... +---+ ExecutableN |
|             |               |             |
+-------------+               +-------------+
~~~

所以，在NV Framework中不需要编写调度代码，只需要继承以上各个单元的对应基类，并在配置函数中注册即可。NV Framework会自动识别你编写的类型并在程序运行时自动创建它，把它放到正确的位置。

## 如何继承/注册

在NV Framework中，你应该像继承普通的C++类一样继承基类。这些基类通常要求你实现一些功能，以便进行统一调用。例如当你需要创建一个处理动作，那么你的类需要继承`IExecutable`基类，`IExecutable`将要求你实现`IsSkip`函数用于判断当前动作是否应当被跳过，以及`Process`函数来执行你的处理动作。它们所需要的函数签名你可以轻易地在文档当中查询到。

一旦你完成了你的功能，你应该将它注册到框架当中，否则我们无法在程序运行时正确地创建它。注册你的类需要提供一个创建函数，以便于我们能够正常读取到你所需要的依赖项。通常而言，框架当中会内置一些通用的创建函数，例如`RegisterStatusType`、`RegisterService`、`RegisterPipeline`等。如果你想要创建一个自定义的创建函数，下面是一个实例：

~~~{.cpp}
std::unique_ptr<StringAppender> CreateStringAppender(AppendContext* context)
{
    return std::make_unique<StringAppender>(context);
}
~~~

它非常简单，以至于我们可以考虑用通用的创建函数来创建，但如果你的初始化涉及配置加载等问题时，通常自定义函数才能够实现你的功能。返回值应该是一个`std::unique_ptr`。参数表中的参数对应了你所需要的依赖，你的依赖会被NV Framework自动生成并在恰当的时候传入到你的创建函数当中（除非它们没有被注册，或者它们也缺少一些依赖导致无法被创建，这种情况下将会在程序运行一开始就抛出异常）

### 基类表

| 单元 | 基类 |
| --- | --- |
| 状态 | `IOAP::BaseStatus` |
| 输入方式 | `IOAP::BaseInputAdapter` |
| 输入数据 | `IOAP::BaseInputData` |
| 处理动作 | `IOAP::IExecutable` |