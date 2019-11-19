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

/**
 * @class	SerialPortIO
 *
 * @brief	串口输出数据服务
 *
 * @author	Xu Zihan
 * @date	2019/11/20
 */
class SerialPortIO : public NautilusVision::IOAP::IService
{
public:

	explicit SerialPortIO()
		:m_SerialPort("COM9",
					  CBR_115200,
					  NautilusVision::IO::SerialPort::EvenParity,
					  8,
					  NautilusVision::IO::SerialPort::OneStopBit,
					  true)
	{
		m_Valid = m_SerialPort.IsOpened();
	}

	/**
	 * @fn	bool SerialPortIO::SendData(float yaw, float pitch, float dist, unsigned int flag)
	 *
	 * @brief	向串口发送数据
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/20
	 *
	 * @param	yaw  	yaw 角度
	 * @param	pitch	pitch 角度
	 * @param	dist 	距离
	 * @param	flag 	标志字节（请使用 FlagHelper 生成）
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	bool SendData(float yaw, float pitch, float dist, unsigned int flag)
	{
		unsigned char buffer[dataLength]{};
		*(buffer + 0) = dataBeginSig;
		*reinterpret_cast<float*>(buffer + 1) = yaw;
		*reinterpret_cast<float*>(buffer + 5) = pitch;
		*reinterpret_cast<float*>(buffer + 9) = dist;
		*reinterpret_cast<unsigned int*>(buffer + 13) = flag;
		
		return m_SerialPort.Send(buffer, sizeof(buffer));
	}

	/**
	 * @fn	unsigned int SerialPortIO::FlagHelper(unsigned char isAimed, unsigned char reserve0, unsigned char reserve1, unsigned char reserve2)
	 *
	 * @brief	生成用于SendData函数的flag参数
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/20
	 *
	 * @param	isAimed 	是否瞄准
	 * @param	reserve0	reserve.
	 * @param	reserve1	reserve.
	 * @param	reserve2	reserve.
	 *
	 * @returns	生成的flag标志.
	 */
	unsigned int FlagHelper(unsigned char isAimed,
							unsigned char reserve0,
							unsigned char reserve1,
							unsigned char reserve2)
	{
		return isAimed << 24 | reserve0 << 16 | reserve1 << 8 | reserve2 << 0;
	}

private:
	static constexpr unsigned char dataBeginSig = 0xA5;
	static constexpr unsigned char dataEndSig = 0xAA;
	static constexpr unsigned int dataLength = 18;

	NautilusVision::IO::SerialPort m_SerialPort;
	bool m_Valid;
};

#endif //_WIN32

#endif //INPUTADAPTER_HPP