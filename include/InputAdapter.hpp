#include <opencv2/opencv.hpp>
#include <nv/IOTypes.hpp>
#include <nv/Service.hpp>
#include <nv/SerialPort.hpp>
#include <spdlog/spdlog.h>

#include "Constants.hpp"
#include "InputModel.hpp"
#include "HKCamera.hpp"

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

#ifdef _WIN32

/**
 * @class	CameraInputSource
 *
 * @brief	摄像头输入
 * 此类提供摄像头输入
 */
class CameraInputSource : public NautilusVision::IOAP::BaseInputAdapter
{
public:
	explicit CameraInputSource(const int camIndex,
							   const int frameWidth,
							   const int frameHeight)
		: m_Camera(camIndex, frameWidth, frameHeight), m_Valid(true)
	{
		m_Camera.StartGrabFrame();
		m_Valid = m_Camera.IsValid();
	}

	CameraInputSource()
		: CameraInputSource(0, ARMOR_FRAME_WIDTH, ARMOR_FRAME_HEIGHT)
	{
	}

	~CameraInputSource()
	{
		try
		{
			m_Camera.Close();
		}
		catch (std::exception e)
		{
			spdlog::error("Camera destory fail");
			std::abort();
		}
	}



	std::shared_ptr<NautilusVision::IOAP::BaseInputData> GetInput() override
	{
		if (!m_Valid)
			return nullptr;
		std::shared_ptr<ImageInputData> result = std::make_shared<ImageInputData>();
		m_Camera.ReadFrame(result->m_Image);
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
						  &CameraInputSource::GetInput,
						  this);
	}

	std::type_index GetInputTypeIndex() const override
	{
		return std::type_index(typeid(ImageInputData));
	}

private:
	HKCamera m_Camera;
	bool m_Valid;
};

class SerialPortIO //[TODO]: Add output pipeline
{
public:
	explicit SerialPortIO()
		:m_SerialPort("COM9",
					  CBR_115200,
					  NautilusVision::IO::SerialPort::EvenParity,
					  8,
					  NautilusVision::IO::SerialPort::OneStopBit,
					  false)
	{
		m_Valid = m_SerialPort.IsOpened();
	}

	bool SendData(int aimed, float yaw, float pitch, float dist)
	{
		if (!m_Valid)
			return false;
		Pack(aimed, yaw, pitch, dist);
		return m_SerialPort.Send(sendFrame, sizeof(sendFrame)) == sizeof(sendFrame);
	}

	bool RecvData(int& signal_1, int& signal_2, int& signal_3)
	{
		if (!m_Valid)
			return false;
		memset(recvFrame, 0, sizeof(recvFrame));
		int recvLen = m_SerialPort.Receive(recvFrame, sizeof(recvFrame));

		if (recvLen == 0)
			return false;

		if (recvFrame[0] == 0xA5 && recvFrame[17] == 0xAA)
		{
			UnPack(signal_1, signal_2, signal_3);
			return true;
		}
		else
			return false;

	}
private:
	static constexpr unsigned int dataLength = 18;

	NautilusVision::IO::SerialPort m_SerialPort;
	bool m_Valid;
	unsigned char sendFrame[dataLength];
	unsigned char recvFrame[dataLength];
	union FloatByte  //sizeof == 4
	{
		float value;
		unsigned char ubyte[4];  //uint8_t
	};
	struct VisionData
	{
		FloatByte _yaw;
		FloatByte _pitch;
		FloatByte _dist;
		VisionData(float yaw, float pitch, float dist)
		{
			_yaw.value = yaw;
			_pitch.value = pitch;
			_dist.value = dist;
		}
	};


	void Pack(int aimed, float yaw, float pitch, float dist)
	{
		VisionData sd(yaw, pitch, dist);
		memset(sendFrame, 0, sizeof(sendFrame));
		sendFrame[0] = 0xA5;

		for (int i = 1, j = 0; i <= 4 && j < 4; ++i, ++j)
			sendFrame[i] = sd._yaw.ubyte[j];

		for (int i = 5, j = 0; i <= 8 && j < 4; ++i, ++j)
			sendFrame[i] = sd._pitch.ubyte[j];

		for (int i = 9, j = 0; i <= 12 && j < 4; ++i, ++j)
			sendFrame[i] = sd._dist.ubyte[j];

		sendFrame[13] = (unsigned char)aimed;
		sendFrame[dataLength - 1] = 0xAA;
	}
	void UnPack(int& signal_1, int& signal_2, int& signal_3)
	{
		signal_1 = (int)recvFrame[14];
		signal_2 = (int)recvFrame[15];
		signal_3 = (int)recvFrame[16];
	}
};

#endif //_WIN32

#endif //INPUTADAPTER_HPP