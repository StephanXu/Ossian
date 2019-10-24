#include <opencv2/opencv.hpp>
#include <nv/IOTypes.hpp>
#include <nv/Service.hpp>

#include "InputModel.hpp"

#ifndef INPUTADAPTER_HPP
#define INPUTADAPTER_HPP

/**
 * @brief 视频输入
 * 此类提供视频输入
 */
class VideoInputSource : public NautilusVision::IOAP::BaseInputAdapter
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

    VideoInputSource()
        : VideoInputSource("test.avi")
    {
    }

    std::shared_ptr<NautilusVision::IOAP::BaseInputData> GetInput() override
    {
        if (!m_Valid)
        {
            return nullptr;
        }
        std::shared_ptr<ImageInputData> result = std::make_shared<ImageInputData>();
        m_VideoSource >> result->m_Image;
        if (result->m_Image.empty())
        {
            m_Valid = false;
            return nullptr;
        }
        return result;
    }

    std::future<std::shared_ptr<NautilusVision::IOAP::BaseInputData>> GetInputAsync() override
    {
        return std::async(std::launch::async,
                          &VideoInputSource::GetInput,
                          this);
    }

    std::type_index GetInputTypeIndex() const override
    {
        return std::type_index(typeid(ImageInputData));
    }

private:
    cv::VideoCapture m_VideoSource;
    bool m_Valid;
};

#endif