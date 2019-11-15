/**
 * @file ProgressBar.cpp
 * @author Stephan Xu
 * @brief 进度条的实现
 * @version 0.1
 * @date 2019-09-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ProgressBar.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <functional>
#include <iostream>

ProgressBar::ProgressBar()
    : ProgressBar(0, "Processing")
{
    ;
}

ProgressBar::ProgressBar(int maxCount)
    : ProgressBar(maxCount, "Processing")
{
}

ProgressBar::ProgressBar(int maxCount, std::string message)
{
    SetMessage(message);
    SetMaxCount(maxCount);
}

std::string ProgressBar::Message()
{
    return m_Msg;
}

void ProgressBar::SetMessage(std::string message)
{
    m_Msg = message;
}

int ProgressBar::MaxCount()
{
    return m_MaxCount;
}

void ProgressBar::SetMaxCount(int maxCount)
{
    m_MaxCount = maxCount;
}

std::string ProgressBar::GenerateBar(double value)
{
    std::stringstream ss;
    ss << m_Msg << c_BarPrefix;

    /* calculate params */
    int fullLen{}, unfullCharIndex{}, paddingLen{};
    double fillLen{};
    fillLen = value * c_BarWidth;
    fullLen = static_cast<int>(value * c_BarWidth);                          // full width
    unfullCharIndex = static_cast<int>((fillLen - fullLen) * c_Fill.size()); // which last char
    paddingLen = c_BarWidth - fullLen + 1;                                   // empty length

    /* generate string */
    for (int i{}; i < fullLen; i++)
        ss << c_Fill.back();
    ss << c_Fill[unfullCharIndex];
    if (0 < paddingLen - 1)
        for (int i{}; i < paddingLen; i++)
            ss << c_Padding;
    ss << c_BarSuffix;
    ss << static_cast<int>(value * 100) << "%";
    return ss.str();
}

void ProgressBar::update(int delta)
{
    m_Count += delta;
    std::cout << std::flush << "\r"
              << std::move(GenerateBar(static_cast<double>(m_Count) / static_cast<double>(m_MaxCount)))
              << " " << m_Count << "/" << m_MaxCount;
}
