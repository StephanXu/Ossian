#include "Aimbuff.hpp"

Aimbuff::Aimbuff(ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
	ossian::IOData<AutoAimStatus>* autoAimStatus,
#ifndef VISION_ONLY
	ossian::IOData<RobotStatus>* robotStatus,
	ossian::IOData<ShootData>* shootDataStatus,
#endif // !VISION_ONLY
	ossian::UARTManager* uartManager)

	: m_Config(config)
	, m_AutoAimStatusSender(autoAimStatus)
#ifndef VISION_ONLY
	, m_RobotStatusListener(robotStatus)
	, m_ShootDataListener(shootDataStatus)
#endif // !VISION_ONLY
	, m_UARTManager(uartManager)
{

}

/*
** 类Aimbuff的构造函数(需传入原图srcImage,有默认构造函数)
*/
//Aimbuff::Aimbuff(cv::Mat _srcImage_) {
//	this->srcImage = _srcImage_;
//}

/*
**	类Aimbuff中的图像预处理函数
**	流程:通道分离->红蓝通道作差->阈值化(以100为分界值),获得只含有风车的二值图
*/
void Aimbuff::imgProcess() {
	std::vector<cv::Mat> imgChannels;		//图像颜色通道(B,G,R)
	cv::split(srcImage, imgChannels);	//通道分离,得到3通道分量
	
	//备注：后期可能会使用绿通道替换蓝通道，得看实际场地情况
	//如果装甲板是红色，就用imgChannels[2]-imgChannels[0]
	//如果装甲板是蓝色，就用imgChannels[0]-imgChannels[2]
	this->binaryImage = imgChannels[2] - imgChannels[0];
	
	//阈值化
	threshold(this->binaryImage, this->binaryImage, COLOR_THRESH, 255, cv::THRESH_BINARY);

	static const cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(17, 17));
	cv::morphologyEx(this->binaryImage, this->binaryImage, cv::MORPH_CLOSE, element);
}

/*
**	找出目标靶心
**	流程:findcontours+等级轮廓->找到特征向量为[-1,-1,-1,x]的轮廓->用最小旋转矩形拟合它
*/
cv::RotatedRect Aimbuff::findRect() {
	//是否要在图上画出该轮廓:0=否	1=是
	this->tag = 0;

	//存放每个轮廓对应的点集
	std::vector<std::vector<cv::Point>> contours;

	//每个轮廓对应的特征向量[x,y,m,n]
	std::vector<cv::Vec4i> hierarchy;

	//调用OpenCV库的findcontours函数
	cv::findContours(binaryImage, contours, hierarchy,
		cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());

	//在dstImage上标出目标靶
	for (int i = 0; i < contours.size(); i++)
	{	//目标轮廓对应的向量为[-1, -1, -1, 正整数],即"只有父轮廓"这一特征
		if (hierarchy[i][0] == -1 && hierarchy[i][1] == -1 && hierarchy[i][2] == -1
			&& hierarchy[i][3] >= 0)
		{
			tag = 1;								//表示要在图上框出目标靶
			this->rect = cv::minAreaRect(contours[i]);	//获得目标靶轮廓的最小旋转矩形
			this->center = this->rect.center;			//获得靶心坐标
			break;
		}
	}
	return this->rect;
}


/*
**	最小二乘法拟合靶心的轨迹圆,来获得风车圆心Centroid坐标和半径dRadius
*/
bool Aimbuff::LeastSquaresCircleFitting()
{
	if (!this->cSet.empty())
	{
		int iNum = (int)this->cSet.size();
		if (iNum < 3)	
			return false;
		double X1 = 0.0;
		double Y1 = 0.0;
		double X2 = 0.0;
		double Y2 = 0.0;
		double X3 = 0.0;
		double Y3 = 0.0;
		double X1Y1 = 0.0;
		double X1Y2 = 0.0;
		double X2Y1 = 0.0;

		for (auto iter = cSet.begin(); iter != cSet.end(); ++iter)
		{
			X1 = X1 + (*iter).x;
			Y1 = Y1 + (*iter).y;
			X2 = X2 + (*iter).x * (*iter).x;
			Y2 = Y2 + (*iter).y * (*iter).y;
			X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
			Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
			X1Y1 = X1Y1 + (*iter).x * (*iter).y;
			X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
			X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
		}
		double C = 0.0;
		double D = 0.0;
		double E = 0.0;
		double G = 0.0;
		double H = 0.0;
		double a = 0.0;
		double b = 0.0;
		double c = 0.0;
		C = iNum * X2 - X1 * X1;
		D = iNum * X1Y1 - X1 * Y1;
		E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
		G = iNum * Y2 - Y1 * Y1;
		H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
		double A = 0.0;
		double B = 0.0;
		double R = 0.0;
		A = a / (-2);
		B = b / (-2);
		R = double(sqrt(a * a + b * b - 4 * c) / 2);
		this->Centroid.x = A;
		this->Centroid.y = B;
		this->dRadius = R;
		return true;
	}
	else
		return false;
	return true;
}


/*
**	将图像坐标转换成极坐标后设置转动角度提前量,然后再转化回图像坐标
*/
bool Aimbuff::calc(const cv::Point2f& Centroid, const double& dRadius, cv::RotatedRect& curRect)
{
	static int times = 0;

	// 目标靶的尺寸
	double rect_w = this->rect.size.width;
	double rect_h = this->rect.size.height;

	cv::Size2f predict_rect_size = cv::Size2f(rect_w, rect_h);

	static double former_frame_theta;
	static double later_frame_theta;

	double delta_x, delta_y;
	double tmp_x, tmp_y;
	cv::Point2f c_tmp;

	// 圆心平移至原点的距离，即点的平移距离
	delta_x = this->Centroid.x;
	delta_y = this->Centroid.y;

	// 靶心点(c.x, c.y)坐标与风车圆心(Centroid.x, Centroid.y)之差
	tmp_x = this->center.x - delta_x;
	tmp_y = this->center.y - delta_y;

	// 靶心点在第四象限
	if (tmp_x > 0 && tmp_y > 0) {
		c_tmp.x = tmp_x;
		c_tmp.y = -tmp_y;
	}
	// 靶心点在第一象限
	else if (tmp_x > 0 && tmp_y < 0) {
		c_tmp.x = tmp_x;
		c_tmp.y = -tmp_y;
	}
	// 靶心点在第三象限
	else if (tmp_x < 0 && tmp_y > 0) {
		c_tmp.x = tmp_x;
		c_tmp.y = -tmp_y;
	}
	// 靶心在第二象限 (tmp_x < 0 && tmp_y < 0)
	else {
		c_tmp.x = tmp_x;
		c_tmp.y = -tmp_y;
	}

	double theta;			//靶心与圆心连线段同x轴正方向的夹角
	double new_theta;		//增加固定提前量后的新夹角
	cv::Point2f predict_point;	//设置固定提前量后的预测点

	theta = (atan2(c_tmp.y, c_tmp.x)) * 180 / CV_PI;

	if (times == 0) {
		former_frame_theta = atan2(c_tmp.y, c_tmp.x);
		later_frame_theta = atan2(c_tmp.y, c_tmp.x);
		times++;
	}
	else if (times <= preparing_time) {
		later_frame_theta = atan2(c_tmp.y, c_tmp.x);
		this->rotateDirectionTag = later_frame_theta > former_frame_theta ? 1 : 0;		//1:逆时针;0:顺时针
		former_frame_theta = later_frame_theta;
		times++;
		curRect = this->rect;
		return false;
	}

	// 顺时针
	if (this->rotateDirectionTag == 0) {
		//theta在第一象限的讨论
		if (theta >= 0 && theta <= 90)
		{
			new_theta = (theta - PARAM) * CV_PI / 180;		//new_theta单位是弧度;theta单位是度
			if (theta - PARAM >= 0)
			{
				// 预测点在第一象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				//cout << -(theta - PARAM) << endl;
				curRect = cv::RotatedRect(predict_point, predict_rect_size, -(theta - PARAM));
				return true;
			}
			else if (theta - PARAM < 0)
			{
				// 预测点在第四象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, (-(theta - PARAM) - 90));
				return true;

			}
		}
		//theta在第二象限的讨论
		else if (theta > 90 && theta <= 180)
		{
			new_theta = (theta - PARAM) * CV_PI / 180;
			if (theta - PARAM <= 90)
			{
				// 预测点在第一象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, -(theta - PARAM));
				return true;
			}
			else
			{
				// 预测点在第二象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, (90 - theta + PARAM));
				return true;
			}
		}
		// theta在第三象限
		else if (theta >= -180 && theta <= -90)
		{
			new_theta = (theta - PARAM) * CV_PI / 180;
			if (theta - PARAM < -180)
			{
				// 预测点在第二象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, (90 - theta + PARAM));
				return true;
			}
			else
			{
				// 预测点在第三象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, (180 - (theta - PARAM)));
				return true;
			}
		}
		// theta在第四象限
		else
		{
			new_theta = (theta - PARAM) * CV_PI / 180;
			if (theta - PARAM <= -90)
			{
				// 预测点在第三象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, (180 - (theta - PARAM)));
				return true;
			}
			else
			{
				// 预测点在第四象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, (-(theta - PARAM) - 90));
				return true;
			}
		}
	}
	// 逆时针
	else if (this->rotateDirectionTag == 1) {
		//theta在第一象限的讨论
		if (theta >= 0 && theta <= 90)
		{
			new_theta = (theta + PARAM) * CV_PI / 180;		//new_theta单位是弧度;theta单位是度
			if (theta + PARAM > 90)
			{
				//预测点落在第二象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, (90 - (theta + PARAM)));
				return true;
			}
			else
			{
				//预测点落在第一象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, -(theta + PARAM));
				return true;
			}
		}
		//theta在第二象限的讨论
		else if (theta > 90 && theta <= 180)
		{
			new_theta = (theta + PARAM) * CV_PI / 180;
			if (theta + PARAM <= 180)
			{
				//预测点落在第二象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, (90 - (theta + PARAM)));
				return true;
			}
			else
			{
				//预测点落在第三象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, 180 - (theta + PARAM));
				return true;
			}
		}
		//theta在第三象限
		else if (theta >= -180 && theta <= -90)
		{
			new_theta = (theta + PARAM) * CV_PI / 180;
			if (theta + PARAM <= -90)
			{
				//预测点落在第三象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, 180 - (theta + PARAM));
				return true;
			}
			else
			{
				//预测点落在第四象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, -(theta + PARAM) - 90);
				return true;
			}
		}
		//theta在第四象限
		else
		{
			new_theta = (theta + PARAM) * CV_PI / 180;
			if (theta + PARAM < 0)
			{
				//预测点在第四象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);
				curRect = cv::RotatedRect(predict_point, predict_rect_size, -(theta + PARAM) - 90);
				return true;
			}
			else
			{
				//预测点在第一象限
				predict_point.x = Centroid.x + dRadius * cos(new_theta);
				predict_point.y = Centroid.y - dRadius * sin(new_theta);

				predict_rect_size.width = this->rect.size.height;
				predict_rect_size.height = this->rect.size.width;

				curRect = cv::RotatedRect(predict_point, predict_rect_size, -(theta + PARAM));
				return true;
			}
		}
	}
	// 刚开始还无法作出预测时返回靶心点
	else {
		curRect = this->rect;
		return false;
	}

	return false;
}

/*
** 根据二值图判断当前打了几个靶
*/
unsigned int Aimbuff::judgeShootNum(cv::Mat &binImg)
{
	std::vector<std::vector<cv::Point>> c;
	cv::bitwise_not(binImg, binImg);
	cv::floodFill(binImg, cv::Point2f(0, 0), cv::Scalar(0, 0, 0));
	cv::findContours(binImg, c, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	return c.size();
}

void Aimbuff::drawRect(cv::RotatedRect &rec, cv::Mat dstImage) {
	cv::Point2f boxPoints[4];
	rec.points(boxPoints);
	for (int i = 0; i < 4; i++) {
		cv::line(dstImage, boxPoints[i], boxPoints[(i+1)%4], cv::Scalar(255, 255, 255), 3);
	}
}

std::tuple<double, double> Aimbuff::AngleSolverPinHole(cv::Point2f targetCenter)
{
	const static cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 2356.912254930781, 0, 720.0503254499439,
		0, 2355.816726575812, 585.5372258072762,
		0, 0, 1); //zhf步兵
	const static cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.08587240444212629, 0.5161812300685514,
		0.001353666202103872, -0.0008039696206637027, -2.113948853124402); //zhf步兵

	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double cx = cameraMatrix.at<double>(0, 2);
	double cy = cameraMatrix.at<double>(1, 2);
	cv::Point2f pnt;
	std::vector<cv::Point2f> in;
	std::vector<cv::Point2f> out;
	in.push_back(targetCenter);

	//对像素点去畸变
	undistortPoints(in, out, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
	pnt = out.front();

	//去畸变后的比值
	double rxNew = (pnt.x - cx) / fx;
	double ryNew = (pnt.y - cy) / fy;

	/*dYaw = atan(rxNew);
	dPitch = -atan(ryNew);*/

	return std::make_tuple(atan(rxNew), -atan(ryNew)); // yaw | pitch
}

double Aimbuff::CompensateOffset(const double pitch, const double dist, const double camToGimbalY)
{
	double camera_target_height = dist * sin(pitch);
	double gun_target_height = camera_target_height + camToGimbalY;
	double gun_pitch_tan = gun_target_height / (dist * cos(pitch));

	return atan(gun_pitch_tan);
}

double Aimbuff::CompensateGravity(const double pitch, const double dist, const double bulletSpeed)
{
	double db = dist / 1000.0 / bulletSpeed;
	double compensateGravity_pitch_tan = tan(pitch) + (0.5 * 9.8 * db * db) / cos(pitch);

	return atan(compensateGravity_pitch_tan);
}

void Aimbuff::Process(unsigned char* pImage) {
	if (!pImage)
	{
		SPDLOG_ERROR("Empty Image!");
		return;
	}

	this->srcImage = cv::Mat(1080, 1440, CV_8UC1, pImage);
	cv::cvtColor(this->srcImage, this->srcImage, cv::COLOR_BayerRG2RGB);
	//this->dstImage = (this->srcImage).clone();
	this->imgProcess();
	this->findRect();

	//	返回当前图中的靶子数
	this->startRectNum = this->judgeShootNum(this->binaryImage);

	if (cSet.size() == MAX_QUEUE_LENGTH)
	{
		cSet.pop_front();
	}
	this->cSet.push_back(this->center);

	bool found = false, shootMode = false;
	double deltaYaw = 0, deltaPitch = 0, dist = 0;

	if (!this->shoot_over) {
		//击打风车的初始状态
		if (this->startRectNum == 1 && this->nextRectNum == 4)
		{
			//最小二乘法拟合
			if (this->LeastSquaresCircleFitting())
			{
				//计算出预测点，并画出
				found = this->calc(this->Centroid, this->dRadius, this->predict_rect);
				this->predict_point = this->predict_rect.center;
				//return cv::Point2d(this->predict_point);
			}
		}
		//由于中途击打失误，风车进行新一轮的击打
		else if (this->startRectNum == 1 && this->nextRectNum != 4)
		{
			this->nextRectNum = 4;
			if (this->LeastSquaresCircleFitting())
			{
				//计算出预测点，并画出
				found = this->calc(this->Centroid, this->dRadius, this->predict_rect);
				this->predict_point = this->predict_rect.center;
				//return cv::Point2d(this->predict_point);
			}
		}
		//第一轮击打成功后，后续的击打预测
		else if (this->startRectNum != 1 && this->startRectNum != this->nextRectNum)
		{
			if (this->LeastSquaresCircleFitting())
			{
				//计算出预测点，并画出
				found = this->calc(this->Centroid, this->dRadius, this->predict_rect);
				this->predict_point = this->predict_rect.center;
				//return cv::Point2d(this->predict_point);
			}
		}
		//自初始时刻击中靶心后，为下次射击作出新的拟合与预测
		else if (this->startRectNum == this->nextRectNum && this->startRectNum != 15)
		{
			found = false;
			this->nextRectNum += 3;
			this->cSet.clear();
			//return cv::Point2d(this->predict_point);
		}
		//靶心全部击打完
		else if (this->startRectNum == 15)
		{
			found = false;
			this->shoot_over = true;
			this->cSet.clear();
			//return cv::Point2d(this->predict_point);

		}

	}

	if (found)
	{
		std::tie(deltaYaw, deltaPitch) = AngleSolverPinHole((this->predict_rect).center); //rad

		deltaPitch = CompensateOffset(deltaPitch, DISTANCE_TO_BUFF, CAM_TO_GIMBAL_Y);
		deltaPitch = CompensateGravity(deltaPitch, DISTANCE_TO_BUFF, BULLET_SPEED);

		shootMode = (fabs(deltaPitch) < 0.025 && fabs(deltaYaw) < 0.05);
	}
	else
	{
		deltaYaw = 0;
		deltaPitch = 0;
		shootMode = false;
	}

	m_AutoAimStatus.m_Found = found;
	m_AutoAimStatus.m_FlagFire = shootMode;
	m_AutoAimStatus.m_Pitch = deltaPitch;
	m_AutoAimStatus.m_Yaw = deltaYaw;
	m_AutoAimStatus.m_Dist = 7000;
	m_AutoAimStatusSender->Set(m_AutoAimStatus);

}