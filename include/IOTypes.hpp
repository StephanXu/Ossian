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

/**
 * @brief 机器人状态
 */
class RoboStatus : public BaseStatus
{
};

/**
 * @brief 输入方式基类
 * 数据 BaseInputData 通过该类获得，并可在 Dispatcher 中传递给相匹配的 Pipeline
 */
class BaseInputAdapter
{
public:
    /**
     * @brief 获得输入
     * 以同步方式获得输入，可能引起线程阻塞以等待输入
     * @return std::shared_ptr<InputDataType> 输入数据的指针
     */
    virtual std::shared_ptr<BaseInputData> GetInput() = 0;

    /**
     * @brief 获得输入
     * 以异步方式获得输入
     * @return std::future<std::shared_ptr<InputDataType>> 输入数据指针的 future
     */
    virtual std::future<std::shared_ptr<BaseInputData>> GetInputAsync() = 0;
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
    cv::Mat m_Image;
};

/**
 * @brief 视频输入
 * 此类提供视频输入
 */
class VideoInputSource : public BaseInputAdapter
{
public:
    /**
     * @brief 建立视频输入
     * 如果视频加载失败，对象将被设置为失效，GetInput GetInputAsync 等函数将不可用。
     * @param filename 
     */
    explicit VideoInputSource(std::string filename)
        : m_VideoSource(filename), m_Valid(true)
    {
        if (!m_VideoSource.isOpened())
        {
            m_Valid = false;
            return;
        }
    }

    std::shared_ptr<BaseInputData> GetInput() override
    {
        if (!m_Valid)
        {
            return std::shared_ptr<ImageInputData>{};
        }
        std::shared_ptr<ImageInputData> result{new ImageInputData};
        m_VideoSource >> result->m_Image;
        return result;
    }

    std::future<std::shared_ptr<BaseInputData>> GetInputAsync() override
    {
        return std::async(std::launch::async, GetInput, this);
    }

private:
    cv::VideoCapture m_VideoSource;
    bool m_Valid;
};

} // namespace NautilusVision
#endif