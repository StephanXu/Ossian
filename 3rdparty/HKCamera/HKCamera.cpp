#include <spdlog/spdlog.h>

#include "HKCamera.hpp"

#include <cstring>

HKCamera::HKCamera(const int camIndex, const int frameWidth, const int frameHeight) noexcept
{
    try
    {
        Initialize();
        SetDevice(camIndex);
        SetFrameSize(frameWidth, frameHeight);
    }
    catch (std::exception e)
    {
        m_IsValid = false;
        spdlog::error("HKCamera initialize fail: {}", e.what());
    }
}

HKCamera::HKCamera() noexcept
{
    try
    {
        Initialize();
    }
    catch (std::exception e)
    {
        m_IsValid = false;
    }
}

HKCamera::~HKCamera()
{
    Close();
}

HKCamera::HKCamera(HKCamera &&camera) noexcept
{
    *this = std::move(camera);
}

HKCamera &HKCamera::operator=(HKCamera &&camera) noexcept
{
    m_IsValid = camera.m_IsValid;
    m_FrameHeight = camera.m_FrameHeight;
    m_FrameWidth = camera.m_FrameWidth;
    m_PayloadSize = camera.m_PayloadSize;
    m_Handle = camera.m_Handle;

    stImageInfo = camera.stImageInfo;
    m_DeviceList = camera.m_DeviceList;

    m_Data = std::move(camera.m_Data);
    return *this;
}

bool HKCamera::Initialize()
{
    m_IsValid = false;
    memset(&m_DeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    if (MV_OK != MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_DeviceList))
        throw std::runtime_error("Enum devices fail");
    m_IsValid = true;
}

std::vector<MV_CC_DEVICE_INFO> HKCamera::ListDevices()
{
    if (!m_IsValid)
        return {};
    std::vector<MV_CC_DEVICE_INFO> res;
    if (0 == m_DeviceList.nDeviceNum)
        throw std::runtime_error("Find No Devices");
    for (unsigned int i = 0; i < m_DeviceList.nDeviceNum; ++i)
    {
        MV_CC_DEVICE_INFO *pDeviceInfo = m_DeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
            break;
        }
        res.emplace_back(*pDeviceInfo);
    }
    return res;
}

// select device to connect

void HKCamera::SetDevice(const int camIndex)
{
    if (!m_IsValid)
        return;
    if (m_DeviceList.nDeviceNum == 0)
        throw std::runtime_error("Device Not Found");
    if (camIndex >= m_DeviceList.nDeviceNum)
        throw std::out_of_range("camIndex out of range");

    if (MV_OK != MV_CC_CreateHandle(&m_Handle, m_DeviceList.pDeviceInfo[camIndex]))
        throw std::runtime_error("Create handle fail");
    if (MV_OK != MV_CC_OpenDevice(m_Handle))
        throw std::runtime_error("Open device fail");
    m_IsOpenedDevice = true;

    if (MV_OK != MV_CC_SetEnumValue(m_Handle, "TriggerMode", MV_CAM_TRIGGER_MODE::MV_TRIGGER_MODE_OFF))
        throw std::runtime_error("Set trigger Mode fail");

    // Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    if (MV_OK != MV_CC_GetIntValue(m_Handle, "PayloadSize", &stParam))
        throw std::runtime_error("Get PayloadSize fail");
    m_PayloadSize = stParam.nCurValue;

    if (MV_OK != MV_CC_SetFloatValue(m_Handle, "ExposureTime", 750))
        throw std::runtime_error("Set ExposureTime fail");

    if (MV_OK != MV_CC_SetEnumValue(m_Handle, "PixelFormat", 0x01080009))
        throw std::runtime_error("Set PixelFormat fail");

    /*
    if (MV_OK != MV_CC_SetFloatValue(m_Handle, "Gamma", 0.4))
        throw std::runtime_error("Set Gamma fail");*/
}

void HKCamera::SetReceiveImageCallback(std::function<ImageCallBackType> callback)
{
    m_ImageCallback = callback;
    auto nRet = MV_CC_RegisterImageCallBackEx(m_Handle, ImageCallBack, this);
    if (MV_OK != nRet)
        throw std::runtime_error(fmt::format("RegisterImageCallBack fail {}", nRet));
}

void HKCamera::SetFrameSize(const int width, const int height)
{
    if (!m_IsValid)
        return;
    if (MV_OK != MV_CC_SetIntValue(m_Handle, "Width", width))
        throw std::runtime_error("Set width fail");
    if (MV_OK != MV_CC_SetIntValue(m_Handle, "Height", height))
        throw std::runtime_error("Set height fail");
    if (MV_OK != MV_CC_SetIntValue(m_Handle, "OffsetX", 0))//80  400  84
        throw std::runtime_error("Set OffsetX fail");
    if (MV_OK != MV_CC_SetIntValue(m_Handle, "OffsetY", 0))//220  300 360
        throw std::runtime_error("Set OffsetY fail");

    m_FrameWidth = width;
    m_FrameHeight = height;
}

bool HKCamera::ReadFrame(cv::cuda::GpuMat &outMat)
{
    if (!m_IsValid)
        return false;
    // get one frame from camera with timeout= ? ms
    if (MV_OK != MV_CC_GetOneFrameTimeout(m_Handle, m_Data.get(), m_PayloadSize, &stImageInfo, 100))
        throw std::runtime_error("Get frame fail"); //[ATTENTION]: Which error code represents time out.
#ifdef _DEBUG
    MVCC_FLOATVALUE struFloatValue = { 0 };
    if (MV_OK == MV_CC_GetFloatValue(m_Handle, "ResultingFrameRate", &struFloatValue))
    {
        spdlog::info("Got frame: {}x{} FrameNum: {} FPS: {:<3}",
                     stImageInfo.nWidth,
                     stImageInfo.nHeight,
                     stImageInfo.nFrameNum,
                     struFloatValue.fCurValue);
    }
    else
    {
        return false; //[ATTENTION]: Is that necessary for exiting the function?
    }
#endif
    if (!ConvertDataToMat(&stImageInfo, m_Data.get(), outMat))
        throw std::runtime_error("OpenCV format convert failed");
    return true;
}

void HKCamera::StartGrabFrame()
{
    if (!m_IsValid)
        return;
    // Start grabbing image
    if (MV_OK != MV_CC_StartGrabbing(m_Handle))
        throw std::runtime_error("Start Grabbing fail");

    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    m_Data = std::make_unique<unsigned char[]>(m_PayloadSize);
    if (!m_Data)
        std::bad_alloc();
    m_IsGrabbing = true;
}

void HKCamera::Close()
{
    // Stop grabbing image
    if (m_IsGrabbing && MV_OK != MV_CC_StopGrabbing(m_Handle))
        throw std::runtime_error("Stop Grabbing fail");
    m_IsGrabbing = false;
    // Close device
    if (m_IsOpenedDevice && MV_OK != MV_CC_CloseDevice(m_Handle))
        throw std::runtime_error("ClosDevice fail");
    m_IsOpenedDevice = false;
    // Destroy m_Handle
    if (m_Handle && MV_OK != MV_CC_DestroyHandle(m_Handle))
        throw std::runtime_error("Destroy Handle fail");
    m_Handle = nullptr;
}

bool HKCamera::IsValid()
{
    return m_IsValid;
}

bool HKCamera::ConvertDataToMat(MV_FRAME_OUT_INFO_EX *pstImageInfo, unsigned char *DataBuffer, cv::cuda::GpuMat &refDest)
{
    //if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
    //{
    //	refDest = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, DataBuffer);
    //}
    //else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    //{
    //	RGB2BGR(DataBuffer, pstImageInfo->nWidth, pstImageInfo->nHeight);
    //	refDest = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, DataBuffer);
    //}
    if (pstImageInfo->enPixelType == PixelType_Gvsp_BayerRG8)
    {
        cv::cuda::Stream cudaStream;
        static constexpr size_t hostBufSize = 1440 * 1080 * sizeof(unsigned char);
        unsigned char* devBuf = nullptr;
        cudaMalloc((void**)&devBuf, hostBufSize);
        cudaMemcpy(devBuf, DataBuffer, hostBufSize, cudaMemcpyHostToDevice);
        cv::cuda::GpuMat BayerRG8Src(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, devBuf);
        //cv::cuda::resize(BayerRG8Src, BayerRG8Src, cv::Size(540, 720), 0.0, 0.0, 1, cudaStream);
        cv::cuda::demosaicing(BayerRG8Src, refDest, cv::cuda::COLOR_BayerRG2BGR_MHT, 0,cudaStream);
        cudaFree(devBuf);
        //cv::Mat BayerRG8Src(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, DataBuffer);
        //cv::cvtColor(BayerRG8Src, refDest, cv::COLOR_BayerRG2BGR);
    } else
    {
        throw std::runtime_error("Unsupported pixel format");
    }

    if (!refDest.empty())
    {
        return true;
    }
    return false;
}

int HKCamera::RGB2BGR(unsigned char *pRgbData, unsigned int width, unsigned int height)
{
    if (NULL == pRgbData)
    {
        return MV_E_PARAMETER;
    }
    for (unsigned int j = 0; j < height; j++)
    {
        for (unsigned int i = 0; i < width; i++)
        {
            std::swap(pRgbData[j * (width * 3) + i * 3], pRgbData[j * (width * 3) + i * 3 + 2]);
        }
    }
    return MV_OK;
}
