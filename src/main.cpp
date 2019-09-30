#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <tuple>
#include <sstream>
#include <memory>
#include <future>
#include <thread>
#include "ColorFilter.hpp"
#include "WindmillDetection.hpp"
#include "ProgressBar.hpp"

const std::string videoFilename{"test.avi"};
const std::string videoOutput{"output.avi"};

class ResourceLoadException : public std::runtime_error
{
public:
    ResourceLoadException() : std::runtime_error("ResourceLoadException") {}
};

int WindmillTest()
{
    /* Define color filters and detection object */
    ColorFilter redFilter{{{{170, 100, 100}, {180, 255, 255}},
                           {{0, 100, 100}, {25, 255, 255}}}};
    ColorFilter blueFilter{{{{85, 100, 100}, {135, 255, 255}}}};
    WindmillDetection detection(80, redFilter);

    /* Video capture object */
    cv::VideoCapture videoSource(videoFilename);
    if (!videoSource.isOpened())
    {
        std::cerr << "Load input video file failed";
        throw ResourceLoadException();
    }

    /* Video output object */
    cv::VideoWriter videoWriter;
    videoWriter.open(videoOutput,
                     videoSource.get(cv::CAP_PROP_FOURCC),
                     videoSource.get(cv::CAP_PROP_FPS),
                     {static_cast<int>(videoSource.get(cv::CAP_PROP_FRAME_WIDTH)),
                      static_cast<int>(videoSource.get(cv::CAP_PROP_FRAME_HEIGHT))});
    if (!videoWriter.isOpened())
    {
        std::cerr << "Create output video file failed";
        throw ResourceLoadException();
    }

    /* Ready for reading video stream */
    cv::Mat frame{};
    long long tick{}, pureTick{};
    double currentFPS{}, latency{}, algoLantency{};
    std::stringstream outputTextString;
    ProgressBar progressBar(videoSource.get(cv::CAP_PROP_FRAME_COUNT),
                            "Processing...");
    while (true)
    {
        progressBar.update(1);
        tick = cv::getTickCount();
        videoSource >> frame;
        if (frame.empty())
        {
            break;
        }
        pureTick = cv::getTickCount();
        if (1 == detection.FeedImage(frame))
        {
            /* Draw detection results */
            cv::circle(frame, detection.Targets()[0], 10,
                       cv::Scalar{0, 255, 0}, 2);
            cv::line(frame, detection.Targets()[0], detection.Center(),
                     cv::Scalar{0, 255, 0}, 2);
            for (int i{1}; i < 5; ++i)
            {
                cv::circle(frame, detection.Targets()[i], 10,
                           cv::Scalar{0, 0, 255}, 2);
                cv::line(frame, detection.Targets()[i], detection.Center(),
                         cv::Scalar{0, 0, 255}, 2);
            }
            cv::circle(frame, detection.Center(), detection.Radius(),
                       cv::Scalar{0, 0, 255}, 2);
        }

        /* Calculate lantency and FPS */
        algoLantency = (cv::getTickCount() - pureTick) / cv::getTickFrequency();
        latency = (cv::getTickCount() - tick) / cv::getTickFrequency();
        currentFPS = 1.0f / latency;
        tick = cv::getTickCount();

        /* Print information */
        outputTextString.str("");
        outputTextString << "FPS:" << currentFPS;
        putText(frame, outputTextString.str(), cv::Point(30, 30),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar{0, 255, 0});

        outputTextString.str("");
        outputTextString << "Latency:" << latency * 1000 << "ms";
        putText(frame, outputTextString.str(), cv::Point(30, 60),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar{0, 255, 0});

        outputTextString.str("");
        outputTextString << "Algorithm Lantency:" << algoLantency * 1000 << "ms";
        putText(frame, outputTextString.str(), cv::Point(30, 90),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar{0, 255, 0});

        /* Write output file */
        videoWriter << frame;
    }
    std::cout << std::endl;
    return 0;
}

int main()
{
}