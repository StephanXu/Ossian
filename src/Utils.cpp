
#include <opencv2/opencv.hpp>

#include "Utils.hpp"

namespace Math
{

double PointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2)
{
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

void RegularizeErrAngle(double& angle, char type)
{
	const static double MAX_YAW_DIFF = 5;
	const static double MAX_PITCH_DIFF = 3;
	const static double MIN_ANGLE_DIFF = 0.2;

	if (type == 'y')
	{
		if (angle > MAX_YAW_DIFF) angle = MAX_YAW_DIFF;
		if (angle < -MAX_YAW_DIFF) angle = -MAX_YAW_DIFF;
		if (fabs(angle) < MIN_ANGLE_DIFF) angle = 0;
	}
	else if (type == 'p')
	{
		if (angle > MAX_PITCH_DIFF) angle = MAX_PITCH_DIFF;
		if (angle < -MAX_PITCH_DIFF) angle = -MAX_PITCH_DIFF;
		if (fabs(angle) < MIN_ANGLE_DIFF) angle = 0;
	}
}

} //Math