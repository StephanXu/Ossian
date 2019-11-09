#include <opencv2/opencv.hpp>
#include <nv/IOTypes.hpp>

#ifndef INPUTMODEL_HPP
#define INPUTMODEL_HPP

/**
 * @brief ÕºœÒ ‰»Î
 * OpenCVµƒMatÕºœÒ ‰»Î
 */
class ImageInputData : public NautilusVision::IOAP::BaseInputData
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

    std::unique_ptr<NautilusVision::IOAP::BaseInputData> Clone() const override
    {
        return std::make_unique<ImageInputData>(*this);
    }

    ImageInputData *GetData() override
    {
        return this;
    }

    cv::Mat m_Image;
};

#endif
