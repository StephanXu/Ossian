
#ifndef IOTYPES_HPP
#define IOTYPES_HPP

#include <opencv2/opencv.hpp>

namespace NautilusVision
{

class BaseInput
{
public:
    virtual void CopyTo(BaseInput &Dest) const = 0;
};

class BaseOutput
{
};

class ImageInput : public BaseInput
{
public:
    ImageInput(cv::Mat image)
        : m_Image(image)
    {
    }
    cv::Mat m_Image;
};

class BaseStatus
{
};

class RoboStatus : public BaseStatus
{
};

} // namespace NautilusVision
#endif