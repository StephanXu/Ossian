//////////////////////////////////////////////////////////////////////////
/// COPYRIGHT NOTICE
/// Copyright (c) 2019, 武汉工程大学 Nautilus
/// All rights reserved.
///
/// @file    NautilusVideoCapture.hpp
/// @brief   读取海康威视USB3 Vision协议工业相机的类，画面参数（分辨率，帧率，曝光时间，伽马等）用 MVS 设置
///
/// @version 1.0
/// @author  王司恺
/// @E-mail： wsk417@126.com
/// @date    2019.10.12
///
///  修订说明：
//////////////////////////////////////////////////////////////////////////

#ifndef HK_CAMERA_HPP
#define HK_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>

#include <exception>
#include <vector>
#include <memory>

class HKCamera
{
public:
	
	using ImageCallBackType = void(unsigned char* data, MV_FRAME_OUT_INFO_EX* pFrameInfo);
	
	/**
	 * @fn	HKCamera::HKCamera(const int camIndex, const int frameWidth, const int frameHeight) noexcept;
	 *
	 * @brief	Constructor
	 * 创建HKCamera对象
	 * @author	Xuzih
	 * @date	2019/11/10
	 * @param	camIndex   	Zero-based index of the camera.
	 * @param	frameWidth 	Width of the frame.
	 * @param	frameHeight	Height of the frame.
	 */
	HKCamera(const int camIndex,
			 const int frameWidth,
			 const int frameHeight) noexcept;
	HKCamera() noexcept;
	~HKCamera();

	HKCamera(const HKCamera& camera) = delete;
	HKCamera(HKCamera&& camera) noexcept;

	HKCamera& operator=(HKCamera& camera) = delete;
	HKCamera& operator=(HKCamera&& camera) noexcept;

	/**
	 * @fn	bool HKCamera::Initialize();
	 *
	 * @brief	Initializes this object
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	bool Initialize();

	/**
	 * @fn	std::vector<MV_CC_DEVICE_INFO> HKCamera::ListDevices();
	 *
	 * @brief	返回枚举出的设备信息
	 *
	 * @returns	A std::vector<MV_CC_DEVICE_INFO> 设备信息数组;
	 */
	std::vector<MV_CC_DEVICE_INFO> ListDevices();

	/**
	 * @fn	void HKCamera::SetDevice(const int camIndex);
	 *
	 * @brief	Sets a device
	 *
	 * @param	camIndex	Zero-based index of the camera.
	 */
	void SetDevice(const int camIndex);

	void SetReceiveImageCallback(std::function<ImageCallBackType> callback);
	
	/**
	 * @fn	void HKCamera::SetFrameSize(const int width, const int height);
	 *
	 * @brief	Sets frame size
	 *
	 * @param	width 	The width.
	 * @param	height	The height.
	 */
	void SetFrameSize(const int width, const int height);

	/**
	 * @fn	bool HKCamera::ReadFrame(cv::Mat& outMat);
	 *
	 * @brief	Reads a frame
	 *
	 * @param [out]	outMat	The out matrix.
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	bool ReadFrame(cv::Mat& outMat);

	/**
	 * @fn	void HKCamera::StartGrabFrame();
	 *
	 * @brief	Starts grabbing frame
	 *
	 */
	void StartGrabFrame();

	/**
	 * @fn	void HKCamera::Close();
	 *
	 * @brief	Closes this object
	 * 需要注意，HKCamera的析构函数将调用Close函数，Close函数有可能抛出异常，如果需要
	 * 手动处理请手动捕获其中的异常
	 */
	void Close();

	/**
	 * @fn	bool HKCamera::IsValid();
	 *
	 * @brief	Query if this object is valid
	 * Initialize和构造函数有可能改变Valid状态，如果Valid为false则该对象失效
	 *
	 * @returns	True if valid, false if not.
	 */
	bool IsValid();

private:
	// Basic settings and status
	bool m_IsValid{ false };
	bool m_IsGrabbing{ false };
	bool m_IsOpenedDevice{ false };
	int m_FrameWidth{ 0 };
	int m_FrameHeight{ 0 };
	unsigned int m_PayloadSize{ 0 };
	void* m_Handle{ nullptr };

	// Enum device
	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
	MV_CC_DEVICE_INFO_LIST m_DeviceList;

	// Image data
	std::unique_ptr<unsigned char[]> m_Data{ nullptr };
	
	std::function<ImageCallBackType> m_ImageCallback{};

	/**
	 * @fn	static int HKCamera::RGB2BGR(unsigned char* pRgbData, unsigned int width, unsigned int height)
	 *
	 * @brief	RGB颜色转换为BGR
	 *
	 * @param			pRgbData	If non-null, data buffer of image.
	 * @param 		  	width   	The width.
	 * @param 		  	height  	The height.
	 *
	 * @returns	MV_OK或错误码。（MV_OK为0）
	 */
	static int RGB2BGR(unsigned char* pRgbData,
					   unsigned int width,
					   unsigned int height);

	/**
	 * @fn	static bool HKCamera::ConvertDataToMat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* DataBuffer, cv::Mat& refDest)
	 *
	 * @brief	convert data stream in Mat format
	 *
	 * @exception	std::runtime_error	Raised when a runtime error condition occurs.
	 *
	 * @param			pstImageInfo	If non-null, information describing the image.
	 * @param [in]		DataBuffer  	If non-null, buffer for data.
	 * @param [out]		refDest			The reference matrix destination.
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	static bool ConvertDataToMat(MV_FRAME_OUT_INFO_EX* pstImageInfo,
								 unsigned char* DataBuffer,
								 cv::Mat& refDest);

	static void __stdcall ImageCallBack(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
	{
		reinterpret_cast<HKCamera*>(pUser)->m_ImageCallback(pData, pFrameInfo);
	}
};

#endif //HK_CAMERA_HPP