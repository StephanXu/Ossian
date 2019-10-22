/**
 * @file IOTypes.hpp
 * @author Xu Zihan (stephanxu@foxmail.com)
 * @brief 输入输入类型
 * @version 0.1
 * @date 2019-10-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef IOTYPES_HPP
#define IOTYPES_HPP

#include <opencv2/opencv.hpp>

#include <future>
#include <memory>
#include <string>

namespace NautilusVision
{

/**
 * @brief 数据数据基类
 */
class BaseInputData
{
public:
    virtual std::shared_ptr<BaseInputData> Clone() const = 0;
    virtual BaseInputData *GetData() = 0;
};

/**
 * @brief 输出数据基类
 */
class BaseOutputData
{
};

/**
 * @brief 状态基类
 */
class BaseStatus
{
};

template<typename StatusType>
std::unique_ptr<StatusType> CreateStatus()
{
    static_assert(std::is_base_of<BaseStatus, StatusType>::value, "StatusType should derived from BaseStatus");
    return std::make_unique<StatusType>();
}

/**
 * @brief 机器人状态
 */
class RoboStatus : public BaseStatus
{
};

/**
 * @brief 图像输入
 * OpenCV的Mat图像输入
 */
class ImageInputData : public BaseInputData
{
public:
    ImageInputData() = default;
    ImageInputData(cv::Mat image)
        : m_Image(image)
    {
    }
    ImageInputData(const ImageInputData &input)
        : m_Image(input.m_Image)
    {
    }

    std::shared_ptr<BaseInputData> Clone() const override
    {
        return std::make_shared<ImageInputData>(*this);
    }

    ImageInputData *GetData() override
    {
        return this;
    }

    cv::Mat m_Image;
};



} // namespace NautilusVision
#endif