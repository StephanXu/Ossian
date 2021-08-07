#pragma once

struct AutoAimStatus
{
	bool m_Found;
	bool m_FlagFire;
	double m_Pitch; //rad
	double m_Yaw;   //rad
	double m_Dist;  //mm

	std::chrono::high_resolution_clock::time_point m_Timestamp;
};

namespace Math
{

double PointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2);

void RegularizeErrAngle(double& angle, char type);

} //Math