#include <opencv2/opencv.hpp>
#include <nv/IOTypes.hpp>
#include <nv/SerialPort.hpp>

#ifndef INPUTMODEL_HPP
#define INPUTMODEL_HPP

/**
 * @brief 图像输入
 * OpenCV的Mat图像输入
 */
class ImageInputData : public NautilusVision::IOAP::BaseInputData
{
public:
    ImageInputData() = default;
    ImageInputData(cv::UMat image)
        : m_Image(image)
    {
    }
    ImageInputData(const ImageInputData &input)
        : m_Image(input.m_Image)
    {
    }

    std::unique_ptr<NautilusVision::IOAP::BaseInputData> Clone() const override
    {
        return std::make_unique<ImageInputData>(*this);
    }

    ImageInputData *GetData() override
    {
        return this;
    }

    cv::UMat m_Image;
};

#endif //INPUTMODEL_HPP
