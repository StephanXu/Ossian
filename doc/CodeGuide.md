# 项目编码规范

## 概述

为方便开发，本项目特制定了本编码规范（下称“本规范”），项目下C/C++代码按照均此规范进行。冲突、遗漏或其他语言可参考[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)或其中文版[Google开源项目风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/)先行编写。并提交Issue修订本规范。

本规范风格主要与C#风格相接近，可参考Google开源项目风格指南进行一定改进。在设计接口时的主要参考规范为[C++ Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
    
## 命名规范

- 文件名：`CamelCase`，C/C++头文件后缀为`.h`，源文件为`.cpp`，源文件`.c`
- 接口类：`ICamelCase`
- 抽象类：`BaseCamelCase`
- 方法（函数）名：`CamelCase`
- Get/Set方法名：与方法名相同，省略`Get`
- bool类型方法：`IsCamelCase`，`Enable`
- 局部变量、参数：`camelCase`
- 输出参数：`void GetRect(Rect& outRect);`
- 输入/输出参数：`void Rotate(Vec2d& refVector);`
- 成员变量：`m_CamelCase`或`mCamelCase`
- 宏：`#define MODULENAME_VERSION 0.1`全大写并加模块前缀
- 枚举：`enum class CamelCase`。枚举项目为`CamelCase`

## 文件风格

文件单行不超过120个字符，且应当在文件头部注释该文件内容

## 头文件

头文件引用顺序

1. 当前源文件对应声明的头文件（如Foo.cpp对应Foo.hpp）
2. C语言系统文件
3. C++系统文件
4. 其他库头文件
5. 本项目内的头文件