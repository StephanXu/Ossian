﻿#ifndef AIMBOT_HPP
#define AIMBOT_HPP

#include <ossian/ossian.hpp>
#include <opencv2/opencv.hpp>

#include "InputAdapter.hpp"
#include "Utils.hpp"

#include <atomic>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>



namespace Ioap = ossian::IOAP;
namespace Utils = ossian::Utils;

class Aimbot : public Ioap::IExecutable
{
public:
	static std::unique_ptr<Aimbot> CreateAimbot(Utils::ConfigLoader* config, SerialPortIO* serialPort)
	{
		return std::unique_ptr<Aimbot>(new Aimbot(config, serialPort));
	}

    void Process(Ioap::BaseInputData* input) override;

    bool IsSkip(const Ioap::BaseStatus& refStatus) override
    {
        return m_Valid;
    }
private:
	Aimbot(Utils::ConfigLoader* config, SerialPortIO* serialPort);

    enum class ArmorType
    {
        Big,
        Small,
        Windmill
    };

    class LightBar
    {
    public:
        static double minArea;
        static double ellipseMinAspectRatio;

        LightBar() = default;
        LightBar(const std::vector<cv::Point>& contour) noexcept
        {
            m_Contour = contour;
            m_Ellipse = cv::fitEllipse(contour);
            m_MinorAxis = m_Ellipse.size.width + 1e-3;
            m_LongAxis = m_Ellipse.size.height + 1e-3;
            m_Angle = m_Ellipse.angle;
            m_AreaEllipse = CV_PI * m_MinorAxis / 2 * m_LongAxis / 2;
            m_AreaContour = cv::contourArea(contour);
            m_Solidity = m_AreaContour / m_AreaEllipse;
        }
        ~LightBar() = default;

        /**
        *	@Brief:		判断此轮廓是否为灯条的轮廓
        *               灯条面积大于一定值（排除噪点） && 拟合的椭圆长短轴比例符合灯条的长宽比 && 轮廓凸度符合灯条凸度 && 符合敌方的颜色
        *	@Return:	真或假
        */
        bool IsLegal() const noexcept
        {
            if (m_AreaContour > minArea&&
                m_LongAxis / m_MinorAxis >= ellipseMinAspectRatio)
            {
                return true;
            }
            return false;
        }

        cv::RotatedRect Ellipse() const noexcept
        {
            return m_Ellipse;
        }
        cv::Point2f Center() const noexcept
        {
            return m_Ellipse.center;
        }
        double LongAxis() const noexcept
        {
            return m_LongAxis;
        }
        double MinorAxis() const noexcept
        {
            return m_MinorAxis;
        }
        double Angle() const noexcept
        {
            return m_Angle;
        }
    private:
        std::vector<cv::Point> m_Contour;
        cv::RotatedRect m_Ellipse;
        double m_MinorAxis;
        double m_LongAxis;
        double m_Angle;
        double m_AreaEllipse;
        double m_AreaContour;
        double m_Solidity;
    };

    class Armor
    {
    public:
        Armor() = default;
        Armor(const LightBar& lightBar1, const LightBar& lightBar2) noexcept
        {
            if (lightBar1.Center().x < lightBar2.Center().x)
            {
                m_LeftLight = lightBar1;
                m_RightLight = lightBar2;
            }
            else
            {
                m_LeftLight = lightBar2;
                m_RightLight = lightBar1;
            }
        }
        Armor(const cv::Rect2d& armorRect, ArmorType armorType) noexcept
        {
            m_Vertexes.emplace_back(cv::Point2d(armorRect.x, armorRect.y));
            m_Vertexes.emplace_back(cv::Point2d(armorRect.x + armorRect.width, armorRect.y));
            m_Vertexes.emplace_back(cv::Point2d(armorRect.x + armorRect.width, armorRect.y + armorRect.height));
            m_Vertexes.emplace_back(cv::Point2d(armorRect.x, armorRect.y + armorRect.height));

            m_MinRect = cv::minAreaRect(m_Vertexes);
            m_Center = (armorRect.tl() + armorRect.br()) / 2;
            m_ArmorType = armorType;
        }
        ~Armor() = default;

        static double maxAngleDiff;
        static double maxYDiffRatio;
        static double minXDiffRatio;
        static double maxHeightDiffRatio;

        static double armorMinAspectRatio;
        static double armorMaxAspectRatio;

        static double bigArmorRatio;
        static double smallArmorRatio;

        static double areaNormalizedBase;
        static double sightOffsetNormalizedBase;

        static cv::Point2d frameCenter;

        bool IsLegal()
        {
            double meanLen = (m_LeftLight.LongAxis() + m_RightLight.LongAxis()) / 2;
            double heightDiffRatio = fabs(m_LeftLight.LongAxis() - m_RightLight.LongAxis()) / std::max(m_LeftLight.LongAxis(), m_RightLight.LongAxis());
            double centerXDiffRatio = fabs(m_LeftLight.Center().x - m_RightLight.Center().x) / meanLen;
            double centerYDiffRatio = fabs(m_LeftLight.Center().y - m_RightLight.Center().y) / meanLen;
            double armorAspectRatio = Math::PointDistance(m_LeftLight.Center(), m_RightLight.Center()) / meanLen;
            double angleDiff = fabs(m_LeftLight.Angle() - m_RightLight.Angle());
            if (angleDiff >= 180)
                angleDiff -= 180;
            else if (angleDiff > 170)
                angleDiff = 180 - angleDiff;
            if (angleDiff < maxAngleDiff
                && centerYDiffRatio <= maxYDiffRatio
                && centerXDiffRatio >= minXDiffRatio
                && heightDiffRatio <= maxHeightDiffRatio
                && armorAspectRatio >= armorMinAspectRatio
                && armorAspectRatio <= armorMaxAspectRatio)
            {
                //TODO: 检测装甲板数字

                cv::Point2f leftLightRectPts[4], rightLightRectPts[4];
                m_LeftLight.Ellipse().points(leftLightRectPts);
                m_RightLight.Ellipse().points(rightLightRectPts);

                m_Vertexes.emplace_back(cv::Point2d(leftLightRectPts[2].x, leftLightRectPts[2].y)); //左上
                m_Vertexes.emplace_back(cv::Point2d(rightLightRectPts[2].x, rightLightRectPts[2].y)); //右上
                m_Vertexes.emplace_back(cv::Point2d(rightLightRectPts[0].x, rightLightRectPts[0].y)); //右下
                m_Vertexes.emplace_back(cv::Point2d(leftLightRectPts[0].x, leftLightRectPts[0].y)); //左下

                cv::Point2d center1 = (m_Vertexes[0] + m_Vertexes[2]) / 2.0;
                cv::Point2d center2 = (m_Vertexes[1] + m_Vertexes[3]) / 2.0;
                m_Center = (center1 + center2) / 2.0;
                m_MinRect = cv::minAreaRect(m_Vertexes);

                double armorAspectRatio = Math::PointDistance(m_LeftLight.Center(), m_RightLight.Center()) / meanLen;
                double ratioOffset;

                if (armorAspectRatio >= bigArmorRatio)
                {
                    m_ArmorType = ArmorType::Big;
                    ratioOffset = std::max(bigArmorRatio - armorAspectRatio, 0.0);
                }
                else
                {
                    m_ArmorType = ArmorType::Small;
                    ratioOffset = std::max(smallArmorRatio - armorAspectRatio, 0.0);
                }

                //计算此装甲板的得分。如果有多个装甲板，选择打得分高的 ，得分 = 装甲板的大小 && 装甲板距离视野中心的距离 && 装甲板的视野正对程度
                double scoreSize = std::exp(cv::contourArea(m_Vertexes)) / areaNormalizedBase;
                double sightOffset = Math::PointDistance(m_Center, frameCenter);
                double scoreDist = exp(-sightOffset / sightOffsetNormalizedBase);
                double scoreRotation = -(pow(ratioOffset, 2) + pow(centerYDiffRatio, 2));
                m_Score = scoreSize + scoreDist + scoreRotation;

                return true;
            }

            return false;
        }
        bool operator < (const Armor& other) const
        {
            return m_Score < other.m_Score;
        }
        cv::RotatedRect MinRect() const
        {
            return m_MinRect;
        }
        cv::Point2d Center() const
        {
            return m_Center;
        }
        ArmorType Type() const
        {
            return m_ArmorType;
        }
        std::vector<cv::Point2d> Vertexes() const
        {
            return m_Vertexes;
        }


    private:
        LightBar m_LeftLight;
        LightBar m_RightLight;
        std::vector<cv::Point2d> m_Vertexes;  ///<装甲板的四个顶点，从左上点开始，顺时针排列
        cv::Point2d m_Center;
        ArmorType m_ArmorType;
        double m_Score;
        cv::RotatedRect m_MinRect;
    };

    class PoseSolver
    {
    public:
        //x向右 y向下 z向前
        //mm
        static double cameraToGimbalX;
        static double cameraToGimbalY;
        static double cameraToGimbalZ;
        static double barrelToGimbalY;
        static double rotOverlapLen;

        static double initV;
        static double initK;
        static double gravity;

        PoseSolver(const cv::Rect2d& bbox, ArmorType armorType) : m_BBox(bbox), m_ArmorType(armorType) 
        {
            double rotAngle = -atan2((cameraToGimbalY + barrelToGimbalY), rotOverlapLen);
            m_CamToGblRot << 1,              0,              0,
                             0,  cos(rotAngle),  sin(rotAngle),
                             0, -sin(rotAngle),  cos(rotAngle);

            m_CamToGblTran << cameraToGimbalX, cameraToGimbalY, cameraToGimbalZ;
        }

        //Yaw: 逆时针正 顺时针负 ; Pitch:下负 上正
        void Solve(double& yaw, double& pitch, double& dist, bool EnableGravity)
        {
            PNPSolver();
            Eigen::Vector3d posInGimbal = m_CamToGblRot * m_WorldToCamTran - m_CamToGblTran;
            dist = posInGimbal(2);
            yaw = atan2(posInGimbal(0), posInGimbal(2));

            double alpha = asin(barrelToGimbalY / sqrt(posInGimbal(1) * posInGimbal(1) + posInGimbal(2) * posInGimbal(2)));
            if (posInGimbal(1) < 0) 
            {
                double theta = atan(-posInGimbal(1) / posInGimbal(2));
                pitch = -(alpha + theta);  
            }
            else if (posInGimbal(1) < barrelToGimbalY)
            {
                double theta = atan(posInGimbal(1) / posInGimbal(2));
                pitch = -(alpha - theta);  
            }
            else 
            {
                double theta = atan(posInGimbal(1)/ posInGimbal(2));
                pitch = (theta - alpha);   
            }

            if (EnableGravity)
                pitch += GetPitch(dist/1000, posInGimbal(1)/1000, initV);
                
        }

    private:
        cv::Rect2d m_BBox;
        ArmorType m_ArmorType;
        cv::Mat m_tvec;
        
        static Eigen::Matrix3d m_CamToGblRot;
        static Eigen::Vector3d m_CamToGblTran;
        static Eigen::Vector3d m_WorldToCamTran;

       /**
       * @Brief: 考虑水平方向空气阻力，计算子弹实际的y坐标
       * @Param x: 相机到目标装甲板的距离 单位 m
       * @Param v: 子弹发射速度 单位 m/s
       * @Param angle: 子弹发射的俯仰角 单位 rad
       * @Return: 云台坐标系中，子弹实际落点的y坐标 单位 m
       */
        double BulletModel(double x, double v, double angle)
        {
            double t, y;
            t = static_cast<double>((std::exp(initK * x) - 1) / (initK * v * std::cos(angle)));
            y = static_cast<double>(v * std::sin(angle) * t - gravity * t * t / 2);
            return y;
        }

       /**
       * @Brief: 计算瞄准所需的云台俯仰角变化量
       * @param x: 相机到目标装甲板的距离 单位 m
       * @param y: 云台坐标系中，击打目标点（装甲板中心）的y坐标 单位 m
       * @param v: 子弹发射速度 单位 m/s
       * @return: 云台俯仰角变化量 单位 rad
       */
        double GetPitch(double x, double y, double v)
        {
            double y_temp, y_actual, dy;
            double a;
            y_temp = y;
            // 20次迭代，数值算法
            for (int i = 0; i < 20; ++i)
            {
                a = std::atan2(y_temp, x);
                y_actual = BulletModel(x, v, a);
                dy = y - y_actual;
                y_temp = y_temp + dy;
                if (std::fabs(dy) < 0.001)
                    break;
                //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
            }
            return a;
        }

        void PNPSolver()
        {
			// constants
            const static double smallArmorWidth = 130.0; // mm
            const static double smallArmorHeight = 71.0;
			const static double bigArmorWidth = 210.0;
			const static double bigArmorHeight = 60.0;
            const static std::vector<cv::Point3d> smallArmorVertex =
            {
                cv::Point3d(-smallArmorWidth / 2, -smallArmorHeight / 2, 0),	//tl
                cv::Point3d(smallArmorWidth / 2, -smallArmorHeight / 2, 0),	//tr
                cv::Point3d(smallArmorWidth / 2,  smallArmorHeight / 2, 0),	//br
                cv::Point3d(-smallArmorWidth / 2,  smallArmorHeight / 2, 0)		//bl
            };
            const static std::vector<cv::Point3d> bigArmorVertex =
			{
				cv::Point3d(-bigArmorWidth / 2, -smallArmorHeight / 2, 0),	//tl
				cv::Point3d(bigArmorWidth / 2, -smallArmorHeight / 2, 0),	//tr
				cv::Point3d(bigArmorWidth / 2,  smallArmorHeight / 2, 0),	//br
				cv::Point3d(-bigArmorWidth / 2,  smallArmorHeight / 2, 0)	//bl
			};
            const static cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1763.56659425676, 0, 755.6922965695335,
                                          0, 1764.629234433968, 560.6661455507484,
                                          0, 0, 1);
            const static cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.05267019741741139, 0.05428699162792178,
                                        -0.0002141029821745741, -0.001124320437987154, -0.4855547217023871);
			
			// tmp
			static cv::Mat rvec;
            std::vector<cv::Point2d> targetPts = {
                m_BBox.tl(),
                m_BBox.tl() + cv::Point2d(m_BBox.width,0),
                m_BBox.br(),
                m_BBox.tl() + cv::Point2d(0,m_BBox.height) };
            if (m_ArmorType == ArmorType::Small)
                cv::solvePnP(smallArmorVertex, targetPts, cameraMatrix, distCoeffs, rvec, m_tvec);
            else
                cv::solvePnP(bigArmorVertex, targetPts, cameraMatrix, distCoeffs, rvec, m_tvec);
            cv::cv2eigen(m_tvec, m_WorldToCamTran);
        }
    };

    enum class AlgorithmState
    {
        Detecting,
        Tracking
    };

    bool DetectArmor(const cv::Mat& frame,
                     Armor& outTarget) noexcept
    {
        using OssianConfig::Configuration;
        static int enemyColor = 0 == m_Config->Instance<Configuration>()->mutable_aimbot()->enemycolor();
        static int brightness = m_Config->Instance<Configuration>()->mutable_aimbot()->brightness();
        static int thresColor = m_Config->Instance<Configuration>()->mutable_aimbot()->threscolor();

        const static cv::Mat element3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        const static cv::Mat element5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        cv::Mat grayColor, binaryColor, binary;
        std::vector<cv::Mat> channels;
        cv::split(frame, channels);

        cv::subtract(channels[enemyColor], channels[std::abs(enemyColor - 2)], grayColor);

        cv::threshold(grayColor, binaryColor, thresColor, 255, cv::THRESH_BINARY);
        cv::bitwise_and(brightness, binaryColor, binary);
        cv::dilate(binary, binary, element3);
        cv::erode(binary, binary, element3);
#ifdef _DEBUG
        //cv::imshow("BinaryBrightness", m_BinaryBrightness);
        cv::imshow("BinaryColor", binaryColor);
        cv::imshow("DebugBinary", binary);
        cv::waitKey(10);
#endif // DEBUG

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<LightBar> lightBars;

        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); ++i)
        {
            if (contours[i].size() >= 6)
            {
                LightBar possibleLightBar(contours[i]);

                if (possibleLightBar.IsLegal())
                {
                    lightBars.emplace_back(possibleLightBar);
#ifdef _DEBUG
                    //ellipse(debugFrame, possibleLightBar.Ellipse(), cv::Scalar(255, 255, 0));
#endif // DEBUG

                }
            }
        }

        std::vector<Armor> armors;

        for (size_t i = 0; i < lightBars.size(); ++i)
        {
            for (size_t j = i + 1; j < lightBars.size(); ++j)
            {
                Armor possibleArmor(lightBars[i], lightBars[j]);

                if (possibleArmor.IsLegal())
                    armors.emplace_back(possibleArmor);
            }
        }

        if (!armors.empty())
        {
            std::sort(armors.rbegin(), armors.rend()); //armor[0]是得分最高的装甲板，也就是击打目标
            outTarget = armors[0];
        }

        return !armors.empty();
    }

    bool FindArmor(const cv::Mat& origFrame, cv::Rect2f& armorBBox, ArmorType& armorType)
    {
        bool armorFound{ false };
        Armor target;
        if (m_ArmorState == AlgorithmState::Detecting)
        {
            armorFound = DetectArmor(origFrame, target);
        }

        if (armorFound)
        {
            armorBBox = cv::boundingRect(target.Vertexes());
            armorType = target.Type();
        }

#ifdef _DEBUG
        if (armorFound)
        {
            //ShowTargetArmor(cv::Scalar(62, 255, 192), cv::Scalar(62, 255, 192));
        }
        else
        {
            //cv::rectangle(debugFrame, defaultAimBox, cv::Scalar(250, 255, 245), 2);
            //cv::circle(debugFrame, (defaultAimBox.tl() + defaultAimBox.br()) / 2, 3, cv::Scalar(250, 255, 245), -1);
        }
#endif // _DEBUG

        return armorFound;
    }

    double m_Yaw = 0;
    double m_Pitch = 0;
    std::mutex m_AngleLock;

    std::atomic<AlgorithmState> m_ArmorState = AlgorithmState::Detecting;
    std::atomic_bool m_Valid = false;
    SerialPortIO* m_SerialPort = nullptr;
    Utils::ConfigLoader* m_Config = nullptr;
};

#endif // AIMBOT_HPP