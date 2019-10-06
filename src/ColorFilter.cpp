/**
 * @file ColorFilter.cpp
 * @author Stephan Xu
 * @brief 颜色过滤器的实现
 * @version 0.1
 * @date 2019-09-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ColorFilter.hpp"
#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>

ColorFilter g_Color{{{{170, 100, 100}, {180, 255, 255}},
                     {{0, 100, 100}, {25, 255, 255}}}};

ColorFilter::ColorFilter(
    const std::vector<std::tuple<cv::Scalar, cv::Scalar>> filters)
    : m_Filters(filters)
{
}

ColorFilter::ColorFilter(const ColorFilter &&colorFilter)
{
    m_Filters = std::move(colorFilter.m_Filters);
}

ColorFilter::ColorFilter(const ColorFilter &colorFilter)
{
    m_Filters.assign(colorFilter.m_Filters.begin(),
                     colorFilter.m_Filters.end());
}

cv::Mat ColorFilter::Filter(const cv::Mat &source) const
{
    cv::Mat result{};
    for (auto &&item : m_Filters)
    {
        cv::Mat mask{};
        cv::inRange(source, std::get<0>(item), std::get<1>(item), mask);
        if (result.empty())
        {
            result = mask;
        }
        result += mask;
    }
    return result;
}
