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
Eigen::Vector3d Aimbot::PoseSolver::m_WorldToCamTran;

cv::Point2d Aimbot::Armor::frameCenter(0, 0);

Aimbot::Aimbot(Utils::ConfigLoader* config)
	:m_Valid(false)
    , m_Config(config)
{
    using OssianConfig::Configuration;
    Aimbot::LightBar::minArea = m_Config->Instance<Configuration>()->mutable_aimbot()->lightbarminarea();
    Aimbot::LightBar::ellipseMinAspectRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->lightbarellipseminaspectratio();

    Aimbot::Armor::maxAngleDiff = m_Config->Instance<Configuration>()->mutable_aimbot()->armormaxanglediff();
    Aimbot::Armor::maxYDiffRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armormaxydiffratio();
    Aimbot::Armor::minXDiffRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armorminxdiffratio();
    Aimbot::Armor::maxHeightDiffRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armormaxheightdiffratio();

    Aimbot::Armor::armorMinAspectRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armorminaspectratio();
    Aimbot::Armor::armorMaxAspectRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armormaxaspectratio();

    Aimbot::Armor::bigArmorRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armorbigarmorratio();
    Aimbot::Armor::smallArmorRatio = m_Config->Instance<Configuration>()->mutable_aimbot()->armorsmallarmorratio();

    Aimbot::Armor::areaNormalizedBase = m_Config->Instance<Configuration>()->mutable_aimbot()->areanormalizedbase();
    Aimbot::Armor::sightOffsetNormalizedBase = m_Config->Instance<Configuration>()->mutable_aimbot()->sightoffsetnormalizedbase();

    Aimbot::PoseSolver::cameraToGimbalX = m_Config->Instance<Configuration>()->mutable_posesolver()->cameratogimbalx();
    Aimbot::PoseSolver::cameraToGimbalY = m_Config->Instance<Configuration>()->mutable_posesolver()->cameratogimbaly();
    Aimbot::PoseSolver::cameraToGimbalZ = m_Config->Instance<Configuration>()->mutable_posesolver()->cameratogimbalz();
    Aimbot::PoseSolver::barrelToGimbalY = m_Config->Instance<Configuration>()->mutable_posesolver()->barreltogimbaly();
    Aimbot::PoseSolver::rotOverlapLen = m_Config->Instance<Configuration>()->mutable_posesolver()->rotoverlaplen();
    Aimbot::PoseSolver::initV = m_Config->Instance<Configuration>()->mutable_posesolver()->initv();
    Aimbot::PoseSolver::initK = m_Config->Instance<Configuration>()->mutable_posesolver()->initk();
    Aimbot::PoseSolver::gravity = m_Config->Instance<Configuration>()->mutable_posesolver()->gravity();
    Aimbot::PoseSolver::scaleDist = m_Config->Instance<Configuration>()->mutable_posesolver()->scaledist();

    Aimbot::Armor::frameCenter.x = m_Config->Instance<Configuration>()->mutable_camera()->framewidth();
    Aimbot::Armor::frameCenter.y = m_Config->Instance<Configuration>()->mutable_camera()->frameheight();

    cudaError_t cudaStatus = cudaMallocManaged(&m_pBinary, 1440 * 1080 * sizeof(unsigned char));
    if (cudaStatus != cudaSuccess)         
        spdlog::error("Aimbot: cudaMallocManaged() Failed: {}", cudaStatus);     
    if (!m_pBinary)        
        std::bad_alloc();
}

void Aimbot::Process(unsigned char* pImage)
{
    //[注意]：这里是不安全的使用方法，应当优化
    //ImageInputData* imageInput = dynamic_cast<ImageInputData*>(input);
	
    //cv::Mat origFrame = imageInput->m_Image;
    auto start=std::chrono::high_resolution_clock::now();
    if (!pImage)
    {
        return;
    }

    cv::Rect2d armorBBox;
    ArmorType armorType;
    bool shootMode = false;  //[TODO] 删除，将发弹决策放到电控部分

    bool foundArmor = FindArmor(pImage, armorBBox, armorType);
    double interval = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();
    spdlog::info("@Aimbot=[$ms={}]", interval);
    double deltaYaw = 0, deltaPitch = 0, dist = 0;
    static PoseSolver angleSolver;
    if (foundArmor)
    {
        std::tie(deltaYaw, deltaPitch, dist) = angleSolver.Solve(armorType, armorBBox); //rad, mm

        Math::RegularizeErrAngle(deltaYaw, 'y');
        Math::RegularizeErrAngle(deltaPitch, 'p');
    }
	spdlog::info("Aimbot Status: {}\t{}\t{}", deltaYaw, deltaPitch, dist);
    
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
        spdlog::error("Send data error: {}", e.what());
        //std::abort();
    }*/

#ifdef _DEBUG
    //putText(debugFrame, fmt::format("ms: {:.4f}",ms), cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(173, 205, 249));
    putText(debugFrame, fmt::format("yaw: {:.4f}", sendYaw), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("pitch: {:.4f}", sendPitch), cv::Point(50, 110), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("dist: {:.4f}", dist), cv::Point(50, 130), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("ms: {}", armorType == ArmorType::Small ? "Small" : "Big"), cv::Point(50, 150), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("ms: {}", shootMode ? "Shoot" : "Stop shooting"), cv::Point(220, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    //cv::circle(debugFrame, redDot, 2, cv::Scalar(0, 255, 0), -1);
    imshow("DebugFrame", debugFrame);
    cv::waitKey(10);
#endif // _DEBUG
}
