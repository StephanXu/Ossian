/**
 * @file ColorFilter.hpp
 * @author Stephan Xu
 * @brief 颜色过滤器
 * @version 0.1
 * @date 2019-09-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef COLOR_FILTER_HPP
#define COLOR_FILTER_HPP

#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>

/**
 * @brief 过滤器接口
 * 过滤器类应当满足此接口定义的方法
 */
class IFilter
{
public:
    /**
     * @brief 过滤操作
     * 
     * @param source 需要过滤的图像
     * @return cv::Mat 过滤后的图像
     */
    virtual cv::Mat Filter(const cv::Mat &source) const = 0;
};

/**
 * @brief 颜色过滤器
 * 支持同时过滤多个范围的颜色过滤器
 */
class ColorFilter : public IFilter
{
public:
    /**
     * @brief 建立一个新的 ColorFilter 对象
     * 建立一个支持对多个范围过滤的颜色过滤器。如：
     * {{{170, 100, 100}, {180, 255, 255}},
     * {{0, 100, 100}, {25, 255, 255}}}
     * @param filters 需要过滤的颜色列表
     */
    explicit ColorFilter(
        const std::vector<std::tuple<cv::Scalar, cv::Scalar>> filters);
    explicit ColorFilter(ColorFilter &&colorFilter);
    explicit ColorFilter(ColorFilter &colorFilter);

    cv::Mat Filter(const cv::Mat &source) const override;

private:
    std::vector<std::tuple<cv::Scalar, cv::Scalar>> m_Filters;
};

extern ColorFilter g_Color;

#endif