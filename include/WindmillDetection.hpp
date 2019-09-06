/**
 * @file WindmillDetection.hpp
 * @author Stephan Xu (xuzihanapple@live.com)
 * @brief 大风车检测相关图形接口
 * @version 0.1
 * @date 2019-09-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef WINDMILL_DETECTION_HPP
#define WINDMILL_DETECTION_HPP

#include "ColorFilter.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <queue>

/**
 * @brief 检测相关基类
 * 
 */
class IDetection
{
    virtual int FeedImage(const cv::Mat &image) = 0;
};

/**
 * @brief 大风车检测
 * 
 * 识别、计算和预测大风车轨迹
 * 
 */
class WindmillDetection : public IDetection
{
public:
    /**
     * 构造并初始化一部分参数
     * @param sampleNum 最大采样数量
     * @param colorFilter IFilter 类型的颜色过滤器.
     */
    explicit WindmillDetection(std::size_t sampleNum, ColorFilter &colorFilter);
    explicit WindmillDetection(std::size_t sampleNum, ColorFilter &&colorFilter);

    /**
     * 计算一张图像Process an image to refresh Targets, Center and Radius.
     * 计算一张图像并刷新（当达到最大采样数量时）当前目标坐标、运动圆心和半径信息
     * @retval 1 成功
     */
    int FeedImage(const cv::Mat &image) override;

    const std::vector<cv::Point2f> &Targets();
    const cv::Point2f &Center();
    float Radius();

private:
    std::queue<cv::Point2f> m_Track;
    ColorFilter m_ColorFilter;
    const size_t m_TrackSize;

    std::vector<cv::Point2f> m_Targets;
    cv::Point2f m_Center = {0, 0};
    float m_Radius = 0;

    /*
    * 轨迹累加数据缓存
    * 由 RefreshAccumulateCache 计算
    */
    float m_x = 0.0;
    float m_xSquare = 0.0;
    float m_xCube = 0.0;
    float m_xMultiY = 0.0;
    float m_xMultiYSquare = 0.0;
    float m_xSquareMultiY = 0.0;
    float m_y = 0.0;
    float m_ySquare = 0.0;
    float m_yCube = 0.0;

    /**
     * @brief 刷新轨迹累加数据缓存
     * 增加一个点并删除一个点
     * @param appendDot 新增的点
     * @param removeDot 要移除的点
     */
    void RefreshAccumulateCache(const cv::Point2f &appendDot,
                                const cv::Point2f &removeDot);

    /**
     * @brief 刷新轨迹累加数据缓存
     * 仅增加一个点
     * @param appendDot 新增的点
     */
    void RefreshAccumulateCache(const cv::Point2f &appendDot);

    /**
     * @brief 拟合圆轨迹
     * 通过最小二乘法，使用记录在 m_Track 中的数据拟合出圆轨迹
     * @return std::tuple<cv::Point2f, float> (圆心, 半径)组成的元组
     */
    std::tuple<cv::Point2f, float> FitRound();

    /**
     * @brief 生成打击目标坐标
     * 
     * @param currentTarget 当前打击目标
     * @param center 圆心
     */
    void GenerateTargetPosition(cv::Point2f &currentTarget,
                                cv::Point2f &center);


    /**
     * @brief 用于顶点过滤的状态标记
     * 
     */
    enum class ContoursMark
    {
        ADD, ///< 加入到结果
        IGNORE, ///< 不添加到结果，但会遍历其子轮廓
        SKIP ///< 不添加到结果，不会遍历其子轮廓
    };


    /**
     * @brief 遍历轮廓树
     * 遍历由 cv::findContours 返回的轮廓树
     * @param contours 轮廓树
     * @param hierarchy 层级关系
     * @param callback 遍历过程回调
     * @return std::vector<int> 结果
     */
    std::vector<int> EnumContours(
        const std::vector<std::vector<cv::Point>> &contours,
        const std::vector<cv::Vec4i> &hierarchy,
        std::function<ContoursMark(const std::vector<cv::Point> &, int)> callback)
        const;

    int EnumContoursHelper(
        std::vector<int> &outContoursIndex,
        const std::vector<std::vector<cv::Point>> &contours,
        const std::vector<cv::Vec4i> &hierarchy,
        const int layer,
        const int index,
        std::function<ContoursMark(const std::vector<cv::Point> &, int)> callback)
        const;
};

#endif