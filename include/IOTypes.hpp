
#ifndef INPUT_HPP
#define INPUT_HPP

#include <opencv2/opencv.hpp>

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


#endif