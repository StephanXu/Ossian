/**
 * @file WindmillDetection.cpp
 * @author Stephan Xu
 * @brief 大风车检测的实现
 * @version 0.1
 * @date 2019-09-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <opencv2/opencv.hpp>
#include <nv/nv.hpp>

#include "WindmillDetection.hpp"
#include "ColorFilter.hpp"
#include "InputModel.hpp"

#include <vector>
#include <tuple>
#include <functional>
#include <algorithm>
#include <atomic>

WindmillDetection::WindmillDetection(std::size_t sampleNum,
                                     ColorFilter &colorFilter)
    : m_Valid(false),
      m_ColorFilter(colorFilter),
      m_TrackSize(sampleNum),
      m_Targets(5)
{
}

WindmillDetection::WindmillDetection(std::size_t sampleNum,
                                     ColorFilter &&colorFilter)
    : WindmillDetection(sampleNum, colorFilter)
{
}

void WindmillDetection::Process(NautilusVision::IOAP::BaseInputData *input)
{
    //[注意]：这里是不安全的使用方法，应当优化
    ImageInputData *imageInput = dynamic_cast<ImageInputData *>(input);

    cv::Mat image = imageInput->m_Image;
    if (image.empty())
    {
        m_Valid = true;
        return;
    }
    /* 
    * Pre-treatment
    * Convert color space into hsv for filter the color.
    * Get a mask in binary expressing the windmill.
    */
    cv::Mat hsv{};
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask{std::move(m_ColorFilter.Filter(hsv))};
    cv::erode(mask, mask,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)),
              cv::Point2f(-1, -1), 1);
    cv::dilate(mask, mask,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)),
               cv::Point2f(-1, -1), 1);

    /*
    * Get current target position by filting contours.
    */
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,
                     contours,
                     hierarchy,
                     cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE);
    auto filterProc = [](const std::vector<cv::Point> &dots,
                         int layer) -> ContoursMark {
        /* Filter by calculating the area */
        double area{cv::contourArea(dots)};
        if (layer == 0 && area < 1000)
            return ContoursMark::Skip;
        else if (layer == 0 && area > 5000)
            return ContoursMark::Skip;
        else if (layer == 1 && area < 200)
            return ContoursMark::Skip;
        if (layer == 1)
        {
            return ContoursMark::Add;
        }
        return ContoursMark::Ignore;
    };
    std::vector<int> contoursIndex{std::move(EnumContours(contours,
                                                          hierarchy,
                                                          filterProc))};

    if (contoursIndex.size() != 1)
    {
        /* There is only one available target at once */
        return;
    }
    cv::RotatedRect minRect{cv::minAreaRect(contours[contoursIndex[0]])};
    cv::Point2f currentTarget{minRect.center};
    // m_Track.emplace(currentTarget);
    m_Track.Push(currentTarget);

    /*
    * Save the track for predicting
    * All related parameters will be refreshed in RefreshAccuulateCache
    */
    if (m_Track.Size() > m_TrackSize)
    {
        // cv::Point2f removeRecord{m_Track.front()};
        auto removeRecord = m_Track.WaitAndPop();
        RefreshAccumulateCache(currentTarget, *removeRecord);
        // m_Track.pop();
    }
    else
    {
        RefreshAccumulateCache(currentTarget);
        return;
    }
    /*
    * Predict the track and other targets.
    */
    std::tie(m_Center, m_Radius) = FitRound();
    GenerateTargetPosition(currentTarget, m_Center);
}

const std::vector<cv::Point2f> &WindmillDetection::Targets()
{
    return m_Targets;
}

const cv::Point2f &WindmillDetection::Center()
{
    return m_Center;
}

float WindmillDetection::Radius()
{
    return m_Radius;
}

void WindmillDetection::RefreshAccumulateCache(const cv::Point2f &appendDot,
                                               const cv::Point2f &removeDot)
{
    /* for performance, I used raw multiply instead of std::pow */
    m_x = m_x + appendDot.x - removeDot.x;
    m_xSquare = m_xSquare + appendDot.x * appendDot.x - removeDot.x * removeDot.x;
    m_xCube = m_xCube + appendDot.x * appendDot.x * appendDot.x -
              removeDot.x * removeDot.x * removeDot.x;
    m_xMultiY = m_xMultiY + appendDot.x * appendDot.y - removeDot.x * removeDot.y;
    m_xMultiYSquare = m_xMultiY + appendDot.x * appendDot.y * appendDot.y -
                      removeDot.x * removeDot.y * removeDot.y;
    m_xSquareMultiY = m_xSquareMultiY + appendDot.x * appendDot.x * appendDot.y -
                      removeDot.x * removeDot.x * removeDot.y;
    m_y = m_y + appendDot.y - removeDot.y;
    m_ySquare = m_ySquare + appendDot.y * appendDot.y - removeDot.y * removeDot.y;
    m_yCube = m_yCube + appendDot.y * appendDot.y * appendDot.y -
              removeDot.y * removeDot.y * removeDot.y;
}

void WindmillDetection::RefreshAccumulateCache(const cv::Point2f &appendDot)
{
    RefreshAccumulateCache(appendDot, std::move(cv::Point2f(0, 0)));
}

std::tuple<cv::Point2f, float> WindmillDetection::FitRound()
{
    float c_{m_TrackSize * m_xSquare - m_x * m_x};
    float d_{m_TrackSize * m_xMultiY - m_x * m_y};
    float e_{m_TrackSize * m_xCube + m_TrackSize * m_xMultiYSquare -
             (m_xSquare + m_ySquare) * m_x};
    float g_{m_TrackSize * m_ySquare - m_y * m_y};
    float h_{m_TrackSize * m_xSquareMultiY + m_TrackSize * m_yCube -
             (m_xSquare + m_ySquare) * m_y};

    float a{(h_ * d_ - e_ * g_) / (c_ * g_ - d_ * d_)};
    float b{(h_ * c_ - e_ * d_) / (d_ * d_ - g_ * c_)};
    float c{-(m_xSquare + m_ySquare + a * m_x + b * m_y) / m_TrackSize};

    cv::Point2f center{a / (-2), b / (-2)};
    float r{0.5f * static_cast<float>(std::sqrt(a * a + b * b - 4 * c))};
    return std::make_tuple(center, r);
}

void WindmillDetection::GenerateTargetPosition(cv::Point2f &currentTarget,
                                               cv::Point2f &center)
{
    cv::Point2f _ = currentTarget - center;
    cv::Vec2f centerVector{center.x,
                           center.y};
    cv::Vec2f vec{_.x, _.y};
    for (int i{}; i < 5; ++i)
    {
        float angle = 2 * 3.1415926f / 5 * i;

        cv::Matx22f rotateMatrix{std::cos(angle), -std::sin(angle),
                                 std::sin(angle), std::cos(angle)};
        cv::Vec2f new_vec = rotateMatrix * vec + centerVector;
        m_Targets[i] = cv::Point2f(new_vec.val[0], new_vec.val[1]);
    }
}

int WindmillDetection::EnumContoursHelper(
    std::vector<int> &outContoursIndex,
    const std::vector<std::vector<cv::Point>> &contours,
    const std::vector<cv::Vec4i> &hierarchy,
    const int layer,
    const int index,
    std::function<ContoursMark(const std::vector<cv::Point> &, int)> callback)
    const
{
    /* validate for collecting */
    ContoursMark filterMark{callback(contours[index], layer)};
    if (ContoursMark::Add == filterMark)
    {
        outContoursIndex.push_back(index);
    }
    else if (ContoursMark::Skip == filterMark)
    {
        return hierarchy[index][0];
    }

    int nextChild = hierarchy[index][2];
    while (nextChild != -1)
    {
        nextChild = EnumContoursHelper(outContoursIndex,
                                       contours,
                                       hierarchy,
                                       layer + 1,
                                       nextChild,
                                       callback);
    }
    return hierarchy[index][0];
}

std::vector<int> WindmillDetection::EnumContours(
    const std::vector<std::vector<cv::Point>> &contours,
    const std::vector<cv::Vec4i> &hierarchy,
    std::function<ContoursMark(const std::vector<cv::Point> &, int)> callback)
    const
{
    int nextChild{0};
    std::vector<int> contoursIndex;
    /* Find the first 0 layer contour */
    while (hierarchy[nextChild][3] != -1)
    {
        nextChild++;
    }
    while (nextChild != -1)
    {
        nextChild = EnumContoursHelper(contoursIndex,
                                       contours,
                                       hierarchy,
                                       0,
                                       nextChild,
                                       callback);
    }
    return contoursIndex;
}