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


/**
 * @brief 空输入
 * 虚拟的无输入
 */
class FakeData : public NautilusVision::IOAP::BaseInputData
{
public:
    FakeData() = default;
    FakeData(int x) :data(x) {}
    std::unique_ptr<NautilusVision::IOAP::BaseInputData> Clone() const override
    {
        return std::make_unique<FakeData>(*this);
    }

    FakeData* GetData() override
    {
        return this;
    }

    int data;
};

#endif //INPUTMODEL_HPP
