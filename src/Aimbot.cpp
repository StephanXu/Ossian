#include "Aimbot.hpp"

#include <opencv2/opencv.hpp>
#include <fmt/format.h>

double Aimbot::LightBar::minArea = 0.0;
double Aimbot::LightBar::ellipseMinAspectRatio = 0.0;

double Aimbot::Armor::maxAngleDiff = 0.0;
double Aimbot::Armor::maxYDiffRatio = 0.0;
double Aimbot::Armor::minXDiffRatio = 0.0;
double Aimbot::Armor::maxHeightDiffRatio = 0.0;

double Aimbot::Armor::armorMinAspectRatio = 0.0;
double Aimbot::Armor::armorMaxAspectRatio = 0.0;

double Aimbot::Armor::bigArmorRatio = 0.0;
double Aimbot::Armor::smallArmorRatio = 0.0;

double Aimbot::Armor::areaNormalizedBase = 0.0;
double Aimbot::Armor::sightOffsetNormalizedBase = 0.0;

double Aimbot::PoseSolver::cameraToGimbalX = 0.0;
double Aimbot::PoseSolver::cameraToGimbalY = 0.0;
double Aimbot::PoseSolver::cameraToGimbalZ = 0.0;
double Aimbot::PoseSolver::barrelToGimbalY = 0.0;
double Aimbot::PoseSolver::rotOverlapLen = 0.0;
double Aimbot::PoseSolver::initV = 0.0;
double Aimbot::PoseSolver::initK = 0.0;
double Aimbot::PoseSolver::gravity = 0.0;
double Aimbot::PoseSolver::scaleDist = 0.0;
double Aimbot::PoseSolver::kalmanDt = 0.0;
Eigen::Vector3d Aimbot::PoseSolver::m_WorldToCamTran;

cv::Point2d Aimbot::Armor::frameCenter(0, 0);

Aimbot::Aimbot(ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
               ossian::IOData<AutoAimStatus>* autoAimStatus,
#ifndef VISION_ONLY
               ossian::IOData<RobotStatus>* robotStatus,
               ossian::IOData<ShootData>* shootDataStatus,
#endif // !VISION_ONLY
               ossian::UARTManager* uartManager)
	: m_Valid(false)
    , m_Config(config)
    , m_AutoAimStatusSender(autoAimStatus)
    , m_UARTManager(uartManager)
#ifndef VISION_ONLY
    , m_RobotStatusListener(robotStatus)
    , m_ShootDataListener(shootDataStatus)
#endif // !VISION_ONLY
{
    Aimbot::LightBar::minArea = *m_Config->Instance()->vision->aimbot->lightbarMinArea;
    Aimbot::LightBar::ellipseMinAspectRatio = *m_Config->Instance()->vision->aimbot->lightBarEllipseMinAspectRatio;

    Aimbot::Armor::maxAngleDiff = *m_Config->Instance()->vision->aimbot->armorMaxAngleDiff;
    Aimbot::Armor::maxYDiffRatio = *m_Config->Instance()->vision->aimbot->armorMaxYDiffRatio;
    Aimbot::Armor::minXDiffRatio = *m_Config->Instance()->vision->aimbot->armorMinXDiffRatio;
    Aimbot::Armor::maxHeightDiffRatio = *m_Config->Instance()->vision->aimbot->armorMaxHeightDiffRatio;

    Aimbot::Armor::armorMinAspectRatio = *m_Config->Instance()->vision->aimbot->armorMinAspectRatio;
    Aimbot::Armor::armorMaxAspectRatio = *m_Config->Instance()->vision->aimbot->armorMaxAspectRatio;

    Aimbot::Armor::bigArmorRatio = *m_Config->Instance()->vision->aimbot->armorBigArmorRatio;
    Aimbot::Armor::smallArmorRatio = *m_Config->Instance()->vision->aimbot->armorSmallArmorRatio;

    Aimbot::Armor::areaNormalizedBase = *m_Config->Instance()->vision->aimbot->areaNormalizedBase;
    Aimbot::Armor::sightOffsetNormalizedBase = *m_Config->Instance()->vision->aimbot->sightOffsetNormalizedBase;

    Aimbot::PoseSolver::cameraToGimbalX = *m_Config->Instance()->vision->poseSolver->cameraToGimbalX;
    Aimbot::PoseSolver::cameraToGimbalY = *m_Config->Instance()->vision->poseSolver->cameraToGimbalY;
    Aimbot::PoseSolver::cameraToGimbalZ = *m_Config->Instance()->vision->poseSolver->cameraToGimbalZ;
    Aimbot::PoseSolver::barrelToGimbalY = *m_Config->Instance()->vision->poseSolver->barrelToGimbalY;
    Aimbot::PoseSolver::rotOverlapLen = *m_Config->Instance()->vision->poseSolver->rotOverlapLen;
    Aimbot::PoseSolver::initV = *m_Config->Instance()->vision->poseSolver->initV;
    Aimbot::PoseSolver::initK = *m_Config->Instance()->vision->poseSolver->initK;
    Aimbot::PoseSolver::gravity = *m_Config->Instance()->vision->poseSolver->gravity;
    Aimbot::PoseSolver::scaleDist = *m_Config->Instance()->vision->poseSolver->scaleDist;
    Aimbot::PoseSolver::kalmanDt = *m_Config->Instance()->vision->poseSolver->kalmanDt;

    Aimbot::Armor::frameCenter.x = *m_Config->Instance()->vision->camera->frameWidth;
    Aimbot::Armor::frameCenter.y = *m_Config->Instance()->vision->camera->frameHeight;

    m_AutoAimFilters[0].SetState(0.25, 0.03);
    m_AutoAimFilters[1].SetState(0.25, 0.03);

#ifdef WITH_CUDA
    /*cudaError_t cudaStatus = cudaMallocManaged(&m_pBinary, 1440 * 1080 * sizeof(unsigned char));
    if (cudaStatus != cudaSuccess)         
        SPDLOG_ERROR("Aimbot: cudaMallocManaged() Failed: {}", cudaStatus);    */ 
    //cuda初始化
    int deviceCount = 0;
    cudaError_t cudaStatus;
    cudaStatus = cudaGetDeviceCount(&deviceCount);
    if (cudaStatus != cudaSuccess)
        throw std::runtime_error("cudaGetDeviceCount() failed");
    if (deviceCount < 1)
        throw std::runtime_error("cuda device not found");
    SPDLOG_TRACE("cudaEnabledDeviceCount={}", deviceCount);
    cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
    cudaSetDeviceFlags(cudaDeviceMapHost);
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess)
        throw std::runtime_error("cudaSetDevice() failed");

    cudaStatus = cudaMalloc(&m_pdFrame, 1440 * 1080 * 1);
    if (cudaStatus != cudaSuccess)
        SPDLOG_ERROR("Aimbot: cudaMalloc() Failed: {}", cudaStatus);    
    m_phBinary = new unsigned char[kGpuMatStep * 1080 * 1];
    if (!m_pdFrame || !m_phBinary)        
        std::bad_alloc();
#endif //WITH_CUDA
}

void Aimbot::Process(unsigned char* pImage)
{
    //[注意]：这里是不安全的使用方法，应当优化
    //ImageInputData* imageInput = dynamic_cast<ImageInputData*>(input);
	
    //cv::Mat origFrame = imageInput->m_Image;

    /*static int cnt = 0;
    std::cerr << "In Aimbot #" <<cnt++<< std::endl;*/
    //return;

    auto start=std::chrono::high_resolution_clock::now();
    if (!pImage)
    {
        SPDLOG_ERROR("Empty Image!");
        return;
    }

    cv::Rect2d armorBBox;
    ArmorType armorType;
    bool shootMode = false;  //[TODO] 删除，将发弹决策放到电控部分

    bool foundArmor = FindArmor(pImage, armorBBox, armorType);
    double procInterval = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();
    
    double deltaYaw = 0, deltaPitch = 0, dist = 0;
    static PoseSolver angleSolver;

    if (foundArmor)
    {
#ifdef VISION_ONLY
        double bulletSpeed = Aimbot::PoseSolver::initV;
#else
        double bulletSpeed = m_ShootDataListener->Get().m_BulletSpeed;
        if (fabs(bulletSpeed) <= DBL_EPSILON)
        {
            bulletSpeed = 15;
        }
#endif // VISION_ONLY

        std::tie(deltaYaw, deltaPitch, dist) = angleSolver.Solve(armorType, armorBBox, true, bulletSpeed); //rad, mm
        /*deltaPitch = m_AutoAimFilters[0].Calc(deltaPitch);
        deltaYaw = m_AutoAimFilters[1].Calc(deltaYaw);*/

        /*if (armorType == ArmorType::Big)
        {
            shootMode = (fabs(deltaPitch) < 0.025 && fabs(deltaYaw) < 0.05);
        }
        else if (armorType == ArmorType::Small)
        {
            shootMode = (fabs(deltaPitch) < 0.025 && fabs(deltaYaw) < 0.025);
        }*/
        
        if (armorType == ArmorType::Big)
        {
            shootMode = (fabs(deltaPitch) < 0.05 && fabs(deltaYaw) < 0.1);
        }
        else if (armorType == ArmorType::Small)
        {
            shootMode = (fabs(deltaPitch) < 0.05 && fabs(deltaYaw) < 0.05);
        }

        /*deltaPitch = DeadbandLimit(deltaPitch, 0.05);
        deltaYaw = DeadbandLimit(deltaYaw, 0.05);*/
        /*Math::RegularizeErrAngle(deltaYaw, 'y');
        Math::RegularizeErrAngle(deltaPitch, 'p');*/
    }
    else
    {
        deltaYaw = 0, deltaPitch = 0, dist = 0;
        m_AutoAimFilters[0].Reset();
        m_AutoAimFilters[1].Reset();
    }
    SPDLOG_TRACE("@Aimbot=[$ms={},$found={},$pitch={},$yaw={},$dist={}]", procInterval, (int)foundArmor, deltaPitch, deltaYaw, dist);

    //std::cerr << "Aimbot: " << foundArmor << '\t' << deltaPitch << '\t' << deltaYaw << std::endl;
    
#ifdef VISION_ONLY
    m_AimbotPLCSendMsg.m_Pitch = deltaPitch * kRadToDegreeCoef * 1000.0;
    m_AimbotPLCSendMsg.m_Yaw = deltaYaw * kRadToDegreeCoef * 1000.0;
    m_AimbotPLCSendMsg.m_FlagFound = foundArmor;
    m_AimbotPLCSendMsg.m_FlagFire = shootMode;
    std::memcpy(m_PLCSendBuf, &m_AimbotPLCSendMsg, kSendBufSize);
    m_UARTManager->WriteTo(m_PLCDevice.get(), sizeof(AimbotPLCSendMsg), m_PLCSendBuf);
#else
    m_AutoAimStatus.m_Found = foundArmor;
    m_AutoAimStatus.m_FlagFire = shootMode;
    m_AutoAimStatus.m_Pitch = deltaPitch;
    m_AutoAimStatus.m_Yaw = deltaYaw;
    m_AutoAimStatus.m_Dist = dist;
    m_AutoAimStatus.m_Timestamp = start;
    m_AutoAimStatusSender->Set(m_AutoAimStatus);
#endif // VISION_ONLY
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //[TODO] 发送两角度给云台
	/*try
	{
        std::lock_guard<std::mutex> guard{ m_AngleLock };
        m_Yaw = m_Yaw + yaw_measured;
        m_Pitch = m_Pitch + pitch_measured;
		m_SerialPort->Commit(m_Yaw,
							 m_Pitch,
							 dist,
							 SerialPortIO::FlagHelper(foundArmor, shootMode, 0, 0));
	}
    catch (std::runtime_error & e)
    {
        SPDLOG_ERROR("Send data error: {}", e.what());
        //std::abort();
    }*/

#ifdef _DEBUG
    putText(debugFrame, fmt::format("ms: {:.2f}", interval), cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(173, 205, 249));
    //cv::rectangle(debugFrame, armorBBox, cv::Scalar(62, 255, 192), 2);
    putText(debugFrame, fmt::format("Yaw: {:.2f}", deltaYaw), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("Pitch: {:.2f}", deltaPitch), cv::Point(50, 110), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("dist: {:.2f}", dist), cv::Point(50, 130), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("Type: {}", armorType == ArmorType::Small ? "Small" : "Big"), cv::Point(50, 150), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("Gun: {}", shootMode ? "Shoot" : "Stop Shooting"), cv::Point(220, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    //cv::circle(debugFrame, redDot, 2, cv::Scalar(0, 255, 0), -1);
    imshow("DebugFrame", debugFrame);
    cv::waitKey(10);
#endif // _DEBUG
}
