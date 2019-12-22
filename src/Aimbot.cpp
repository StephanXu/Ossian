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

double Aimbot::PoseSolver::offsetZPitch = 0.0;
double Aimbot::PoseSolver::offsetYPitch = 0.0;
double Aimbot::PoseSolver::offsetZYaw = 0.0;
double Aimbot::PoseSolver::offsetX = 0.0;
double Aimbot::PoseSolver::initV = 0.0;
double Aimbot::PoseSolver::initK = 0.0;
double Aimbot::PoseSolver::gravity = 0.0;

cv::Point2f Aimbot::Armor::frameCenter(0, 0);

Aimbot::Aimbot(Utils::Configuration* config, SerialPortIO* serialPort)
	:m_Valid(false)
    , m_Config(config)
    , m_SerialPort(serialPort)
{
    Aimbot::LightBar::minArea = m_Config->LoadDoubleValue("/aimbot/lightBar/minArea");
    Aimbot::LightBar::ellipseMinAspectRatio = m_Config->LoadDoubleValue("/aimbot/lightBar/ellipseMinAspectRatio");

    Aimbot::Armor::maxAngleDiff = m_Config->LoadDoubleValue("/aimbot/armor/maxAngleDiff");
    Aimbot::Armor::maxYDiffRatio = m_Config->LoadDoubleValue("/aimbot/armor/maxYDiffRatio");
    Aimbot::Armor::minXDiffRatio = m_Config->LoadDoubleValue("/aimbot/armor/minXDiffRatio");
    Aimbot::Armor::maxHeightDiffRatio = m_Config->LoadDoubleValue("/aimbot/armor/maxHeightDiffRatio");

    Aimbot::Armor::armorMinAspectRatio = m_Config->LoadDoubleValue("/aimbot/armor/armorMinAspectRatio");
    Aimbot::Armor::armorMaxAspectRatio = m_Config->LoadDoubleValue("/aimbot/armor/armorMaxAspectRatio");

    Aimbot::Armor::bigArmorRatio = m_Config->LoadDoubleValue("/aimbot/armor/bigArmorRatio");
    Aimbot::Armor::smallArmorRatio = m_Config->LoadDoubleValue("/aimbot/armor/smallArmorRatio");

    Aimbot::Armor::areaNormalizedBase = m_Config->LoadDoubleValue("/aimbot/areaNormalizedBase");
    Aimbot::Armor::sightOffsetNormalizedBase = m_Config->LoadDoubleValue("/aimbot/sightOffsetNormalizedBase");

    Aimbot::PoseSolver::offsetZPitch = m_Config->LoadDoubleValue("/poseSolver/offsetZPitch");
    Aimbot::PoseSolver::offsetYPitch = m_Config->LoadDoubleValue("/poseSolver/offsetYPitch");
    Aimbot::PoseSolver::offsetZYaw = m_Config->LoadDoubleValue("/poseSolver/offsetZYaw");
    Aimbot::PoseSolver::offsetX = m_Config->LoadDoubleValue("/poseSolver/offsetX");
    Aimbot::PoseSolver::initV = m_Config->LoadDoubleValue("/poseSolver/initV");
    Aimbot::PoseSolver::initK = m_Config->LoadDoubleValue("/poseSolver/initK");
    Aimbot::PoseSolver::gravity = m_Config->LoadDoubleValue("/poseSolver/gravity");

    Aimbot::Armor::frameCenter.x = m_Config->LoadIntegerValue("/camera/frameWidth") / 2;
    Aimbot::Armor::frameCenter.y = m_Config->LoadIntegerValue("/camera/frameHeight") / 2;
}

void Aimbot::Process(Ioap::BaseInputData* input)
{
    const static cv::Point2f redDot(744, 642); //步兵
    const double maxShootRadius = m_Config->LoadDoubleValue("/aimbot/maxShootRadius");
    const double offsetYaw = m_Config->LoadDoubleValue("/poseSolver/offsetYaw");
    const double offsetPitch = m_Config->LoadDoubleValue("/poseSolver/offsetPitch");
    static KalmanFilter kf(4, 2);

    //[注意]：这里是不安全的使用方法，应当优化
    ImageInputData* imageInput = dynamic_cast<ImageInputData*>(input);
	
    cv::UMat origFrame = imageInput->m_Image;
    if (origFrame.empty())
    {
        m_Valid = true;
        return;
    }
#ifdef _DEBUG
	cv::imshow("damn", origFrame);
	cv::waitKey(1);
#endif
    cv::Rect2f armorBBox;
    ArmorType armorType;
    bool shootMode = false;

    bool foundArmor = FindArmor(origFrame, armorBBox, armorType);
    float yaw_measured = 0, pitch_measured = 0, dist = 0;
    float yaw_predicted = 0, pitch_predicted = 0;
    if (foundArmor)
    {
        PoseSolver angleSolver(armorBBox, armorType);
        angleSolver.Solve(yaw_measured, pitch_measured, dist); //degree, mm

        Math::RegularizeErrAngle(yaw_measured, 'y');
        Math::RegularizeErrAngle(pitch_measured, 'p');

        kf.PredictAndCorrect(yaw_predicted, pitch_predicted, yaw_measured, pitch_measured);
        Math::RegularizeErrAngle(yaw_predicted, 'y');
        Math::RegularizeErrAngle(pitch_predicted, 'p');

        if (Math::PointDistance(redDot, (armorBBox.tl() + armorBBox.br()) / 2) < maxShootRadius)
            shootMode = true;
        else
            shootMode = false;
    }
    else
    {
        kf.Init();
        shootMode = false;
    }

    float sendYaw = foundArmor ? yaw_predicted + offsetYaw : 0;
    float sendPitch = foundArmor ? pitch_predicted + offsetPitch : 0;
    Math::RegularizeErrAngle(sendYaw, 'y');
    Math::RegularizeErrAngle(sendPitch, 'p');

	spdlog::info("Aimbot Status: {}\t{}\t{}", sendYaw, sendPitch, dist);

	try
	{
        std::lock_guard<std::mutex> guard{ m_AngleLock };
        m_Yaw = m_Yaw + sendYaw;
        m_Pitch = m_Pitch + sendPitch;
		m_SerialPort->Commit(m_Yaw,
							 m_Pitch,
							 dist,
							 SerialPortIO::FlagHelper(foundArmor, shootMode, 0, 0));
	}
    catch (std::runtime_error & e)
    {
        spdlog::error("Send data error: {}", e.what());
        //std::abort();
    }

#ifdef _DEBUG
    //putText(debugFrame, fmt::format("ms: {:.4f}",ms), cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(173, 205, 249));
    cv::Mat debugFrame;
    origFrame.copyTo(debugFrame);
    putText(debugFrame, fmt::format("yaw: {:.4f}", sendYaw), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("pitch: {:.4f}", sendPitch), cv::Point(50, 110), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("dist: {:.4f}", dist), cv::Point(50, 130), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("ms: {}", armorType == ArmorType::Small ? "Small" : "Big"), cv::Point(50, 150), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    putText(debugFrame, fmt::format("ms: {}", shootMode ? "Shoot" : "Stop shooting"), cv::Point(220, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 252, 124));
    cv::circle(debugFrame, redDot, 2, cv::Scalar(0, 255, 0), -1);
    imshow("DebugFrame", debugFrame);
    cv::waitKey(10);
#endif // _DEBUG
}
