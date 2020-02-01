
#ifndef INPUTADAPTER_HPP
#define INPUTADAPTER_HPP

#include <opencv2/opencv.hpp>
#include <ossian/IOTypes.hpp>
#include <ossian/Service.hpp>
#include <ossian/io/SerialPort.hpp>
#include <ossian/Factory.hpp>
#include <spdlog/spdlog.h>
#include <HKCamera.hpp>

#include "Config.pb.h"
#include "InputModel.hpp"

namespace Ioap = ossian::IOAP;
namespace Utils = ossian::Utils;
namespace IO = ossian::IO;


/**
 * @brief 视频输入
 * 此类提供视频输入
 */
class VideoInputSource : public Ioap::BaseInputAdapter
{
public:
	/**
	 * @brief 建立视频输入
	 * 如果视频加载失败，对象将被设置为失效，GetInput GetInputAsync 等函数将不可用。
	 * @param filename
	 */
	OSSIAN_SERVICE_SETUP(VideoInputSource(Utils::ConfigLoader* config))
		: m_VideoSource(config->Instance<OssianConfig::Configuration>()->mutable_testvideosource()->filename())
		, m_Valid(true)
	{
		if (!m_VideoSource.isOpened())
		{
			m_Valid = false;
			return;
		}
	}

    std::shared_ptr<Ioap::BaseInputData> GetInput() override
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

    std::future<std::shared_ptr<Ioap::BaseInputData>> GetInputAsync() override
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

class FakeInputSource : public Ioap::BaseInputAdapter
{
public:
	OSSIAN_SERVICE_SETUP(FakeInputSource(Utils::ConfigLoader* config))
		: m_Valid(true)
	{
	}

	std::shared_ptr<Ioap::BaseInputData> GetInput() override
	{
		if (!m_Valid)
		{
			return nullptr;
		}
		std::shared_ptr<FakeData> result = std::make_shared<FakeData>(1);
		return result;
	}

	std::future<std::shared_ptr<Ioap::BaseInputData>> GetInputAsync() override
	{
		return std::async(std::launch::async,
						  &FakeInputSource::GetInput,
						  this);
	}

	std::type_index GetInputTypeIndex() const override
	{
		return std::type_index(typeid(FakeData));
	}

private:
	bool m_Valid;
};

/**
 * @class	CameraInputSource
 *
 * @brief	摄像头输入
 * 此类提供摄像头输入
 */
class CameraInputSource : public Ioap::BaseInputAdapter
{
public:
	OSSIAN_SERVICE_SETUP(CameraInputSource(ossian::Utils::ConfigLoader* config))
		: m_Camera(config->Instance<OssianConfig::Configuration>()->mutable_camera()->deviceindex(),
				   config->Instance<OssianConfig::Configuration>()->mutable_camera()->framewidth(),
				   config->Instance<OssianConfig::Configuration>()->mutable_camera()->frameheight())
		, m_Valid(true)
	{
		try
		{
			m_Camera.StartGrabFrame();
		}
		catch (std::runtime_error & e)
		{
			spdlog::error(e.what());
			m_Valid = false;
		}
		catch (std::bad_alloc & e)
		{
			spdlog::error(e.what());
			m_Valid = false;
		}
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

	std::shared_ptr<Ioap::BaseInputData> GetInput() override
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

	std::future<std::shared_ptr<Ioap::BaseInputData>> GetInputAsync() override
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


#ifdef _WIN32

/**
 * @class	SerialPortIO
 *
 * @brief	串口输出数据服务
 *
 * @author	Xu Zihan
 * @date	2019/11/20
 */
class SerialPortIO : public Ioap::IService
{
public:
	OSSIAN_SERVICE_SETUP(SerialPortIO(Utils::ConfigLoader* config))
		:m_SyncThread(nullptr)
	{
		try
		{
			m_Config = config->Instance<OssianConfig::Configuration>()->mutable_serialport();
			m_SerialPort.Open(m_Config->portname(),
							  m_Config->baudrate(),
							  m_Config->parity(),
							  m_Config->databit(),
							  m_Config->stopbit(),
							  m_Config->synchronize());
			m_Valid = m_SerialPort.IsOpened();
			unsigned int syncInterval = m_Config->syncinterval();
			StartSync(syncInterval);
		}
		catch (std::runtime_error & e)
		{
			spdlog::error(e.what());
			m_Valid = false;
		}

	}

	~SerialPortIO()
	{
		if (m_SyncThread)
		{
			m_SyncThread->join();
		}
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
		//unsigned char buffer[dataLength]{};
		//*(buffer + 0) = dataBeginSig;
		//*reinterpret_cast<float*>(buffer + 1) = yaw;
		//*reinterpret_cast<float*>(buffer + 5) = pitch;
		//*reinterpret_cast<float*>(buffer + 9) = dist;
		//*reinterpret_cast<unsigned int*>(buffer + 13) = flag;
		static OutModel model;
		model.distance = dist;
		model.flags = flag;
		model.pitchAngle = pitch;
		model.yawAngle = yaw;
		return m_SerialPort.Send(reinterpret_cast<unsigned char*>(&model), sizeof(OutModel));
	}

	/**
	 * @fn	static unsigned int SerialPortIO::FlagHelper(unsigned char isAimed, unsigned char reserve0, unsigned char reserve1, unsigned char reserve2)
	 *
	 * @brief	生成用于SendData函数的flag参数
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/20
	 *
	 * @param	isAimed 	是否瞄准.
	 * @param	reserve0	reserve.
	 * @param	reserve1	reserve.
	 * @param	reserve2	reserve.
	 *
	 * @returns	生成的flag标志.
	 */
	static unsigned int FlagHelper(unsigned char isAimed,
								   unsigned char reserve0,
								   unsigned char reserve1,
								   unsigned char reserve2)
	{
		return isAimed << 24 | reserve0 << 16 | reserve1 << 8 | reserve2 << 0;
	}
#ifdef _WIN32
#pragma pack(push,1)
#endif
	struct InModel
	{
		uint8_t beginCode = 0xA5;
		uint32_t gimbalIndex;
		float yawMotor;
		float pitchMotor;
		uint8_t endCode = 0xAA;
	};

	struct OutModel
	{
		uint8_t beginCode = 0xA5;
		float yawAngle;
		float pitchAngle;
		float distance;
		uint32_t flags;
		uint8_t endCode = 0xAA;
	};
#ifdef _WIN32
#pragma pack(pop)
#endif
	void SyncStatus(unsigned int interval)
	{
		static InModel inModel;
		static OutModel outModel;
		while (m_SyncThreadFlag)
		{
			outModel = m_OutModel;
			m_SerialPort.Send(reinterpret_cast<unsigned char*>(&outModel), sizeof(OutModel));

			m_SerialPort.Receive(reinterpret_cast<unsigned char*>(&inModel), sizeof(inModel));
			if (0xA5 == inModel.beginCode && 0xAA == inModel.endCode)
			{
				m_InModel.store(inModel);
				//spdlog::info("damn {}\t{}\t{}\t{}", inModel.beginCode, inModel.gimbalIndex, inModel.pitchMotor, inModel.yawMotor);
			}
			std::this_thread::sleep_for(std::chrono::microseconds(interval));
		}
	}

	void StartSync(unsigned int interval)
	{
		if (m_SyncThreadFlag)
		{
			m_SyncThreadFlag = false;
			if (m_SyncThread)
				m_SyncThread->join();
		}
		m_SyncThread.reset(new std::thread(&SerialPortIO::SyncStatus, this, interval));
		m_SyncThreadFlag = true;
	}

	void Intercept(InModel& outData) const
	{
		outData = m_InModel;
	}

	InModel Intercept() const
	{
		InModel model;
		Intercept(model);
		return model;
	}

	void Commit(OutModel& refData)
	{
		m_OutModel = refData;
	}

	void Commit(float yaw, float pitch, float dist, unsigned int flag)
	{
		static OutModel model;
		model.distance = dist;
		model.flags = flag;
		model.pitchAngle = pitch;
		model.yawAngle = yaw;
		Commit(model);
	}

	void CommitRel(float yaw, float pitch, float absDist, unsigned int flag)
	{
		static OutModel model;
		model = m_OutModel;
		model.yawAngle += yaw;
		model.pitchAngle += pitch;
		model.distance = absDist;
		model.flags = flag;
		Commit(model);
	}

private:
	std::atomic<InModel> m_InModel;
	std::atomic<OutModel> m_OutModel;

	std::unique_ptr<std::thread> m_SyncThread;
	bool m_SyncThreadFlag = false;

	IO::SerialPort m_SerialPort;

	OssianConfig::SerialPort* m_Config = nullptr;

	bool m_Valid;
};

#endif //_WIN32

#ifndef _WIN32

/**
 * @class	SerialPortIO
 *
 * @brief	串口输出数据服务（不能使用！！！！！）
 *
 * @author	Xu Zihan
 * @date	2019/11/20
 */
class SerialPortIO : public Ioap::IService
{
public:
	OSSIAN_SERVICE_SETUP(SerialPortIO(Utils::ConfigLoader* config))
		:m_SyncThread(nullptr)
	{

	}
	~SerialPortIO()
	{
		
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
		
	}

	/**
	 * @fn	static unsigned int SerialPortIO::FlagHelper(unsigned char isAimed, unsigned char reserve0, unsigned char reserve1, unsigned char reserve2)
	 *
	 * @brief	生成用于SendData函数的flag参数
	 *
	 * @author	Xu Zihan
	 * @date	2019/11/20
	 *
	 * @param	isAimed 	是否瞄准.
	 * @param	reserve0	reserve.
	 * @param	reserve1	reserve.
	 * @param	reserve2	reserve.
	 *
	 * @returns	生成的flag标志.
	 */
	static unsigned int FlagHelper(unsigned char isAimed,
								   unsigned char reserve0,
								   unsigned char reserve1,
								   unsigned char reserve2)
	{
		
	}
#ifdef _WIN32
#pragma pack(push,1)
#endif
	struct InModel
	{
		uint8_t beginCode = 0xA5;
		uint32_t gimbalIndex;
		float yawMotor;
		float pitchMotor;
		uint8_t endCode = 0xAA;
	};

	struct OutModel
	{
		uint8_t beginCode = 0xA5;
		float yawAngle;
		float pitchAngle;
		float distance;
		uint32_t flags;
		uint8_t endCode = 0xAA;
	};
#ifdef _WIN32
#pragma pack(pop)
#endif
	void SyncStatus(unsigned int interval)
	{
		
	}

	void StartSync(unsigned int interval)
	{
		
	}

	void Intercept(InModel& outData) const
	{
		
	}

	InModel Intercept() const
	{
		
	}

	void Commit(OutModel& refData)
	{
		
	}

	void Commit(float yaw, float pitch, float dist, unsigned int flag)
	{
		
	}

	void CommitRel(float yaw, float pitch, float absDist, unsigned int flag)
	{
		
	}

private:

	std::unique_ptr<std::thread> m_SyncThread;
	bool m_SyncThreadFlag = false;

	bool m_Valid;
};

#endif //_WIN32

#endif //INPUTADAPTER_HPP