#pragma once
#include <ossian/Factory.hpp>
#include <ossian/Configuration.hpp>
#include <opencv2/opencv.hpp>
#include <ossian/IOData.hpp>
#include <ossian/io/UART.hpp>
#include <CtrlAlgorithms.hpp>

#include "InputAdapter.hpp"
#include "Utils.hpp"
#include "Referee.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>

#include <vector>
#include <deque>
#include <cstdio>
#include <cmath>
#include <cmath>

constexpr int COLOR_THRESH = 80;
constexpr double PARAM = 30;				//提前量角度(单位:度)
constexpr int preparing_time = 240;	// 旋转方向测试时间
constexpr int MAX_QUEUE_LENGTH = 50;

constexpr double CAM_TO_GIMBAL_Y = -41.80;  // mm
constexpr double DISTANCE_TO_BUFF = 7000; // mm
constexpr double BULLET_SPEED = 29; // m/s

class Aimbuff : public ossian::IODataBuilder<std::mutex, AutoAimStatus>
{
public:
	cv::Mat srcImage;				//要传入的原图
	cv::Mat binaryImage/*, dstImage*/;	//中间过程的二值图(用于确定射击次数)，和用于画上标记的最终图
	
	int myColor = 0;			//装甲板颜色，默认值:0(代表未知)，蓝色:0，红色:2

	cv::RotatedRect rect;			//目标靶的Rect
	cv::RotatedRect predict_rect;	//预测靶子
	bool tag;						//1:找到靶子		0:没有找到靶子	(如果你需要在图上画出来目标靶的话)
	cv::Point2f center;					//目标矩形靶心(imgprocess函数中获得)
	
	std::deque<cv::Point2f> cSet;		//一系列靶心的存储容器，用于最小二乘法拟合圆周
	cv::Point2f Centroid;			//风车圆心
	double dRadius;					//风车圆周半径
	
	cv::Point2f predict_point;			//预测点
	short int rotateDirectionTag = -1;	//风车旋转方向	未旋转:-1	顺时针:0		逆时针:1
	
	//击打过程轮廓数1->4->7->10->13->15
	bool shoot_over = false;		//五个靶子是否打完(默认为false)
	unsigned int startRectNum;		//初始时刻轮廓数量
	unsigned int nextRectNum = 4;	//紧接初始时刻的下一时刻应出现的轮廓数量

	OSSIAN_SERVICE_SETUP(Aimbuff(ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		ossian::IOData<AutoAimStatus>* autoAimStatus,
#ifndef VISION_ONLY
		ossian::IOData<RobotStatus>* robotStatus,
		ossian::IOData<ShootData>* shootDataStatus,
#endif // !VISION_ONLY
		ossian::UARTManager* uartManager
		));
	//Aimbuff(cv::Mat _srcImage_);			//重载构造函数，将原图传入

	void imgProcess();					//获得图形二值图
	cv::RotatedRect findRect();			//找到目标靶
	bool LeastSquaresCircleFitting();	//最小二乘法拟合靶心运动的轨迹圆
	bool calc(const cv::Point2f& Centroid, const double& dRadius, cv::RotatedRect& curRect);	//极坐标预测法
	unsigned int judgeShootNum(cv::Mat &binImg);	//根据二值图判断当前打到第几个靶\

	~Aimbuff() {};	//析构函数

	void drawRect(cv::RotatedRect &rec, cv::Mat dstImage);

	std::tuple<double, double> AngleSolverPinHole(cv::Point2f targetCenter);
	double CompensateOffset(const double pitch, const double dist, const double camToGimbalY);
	double CompensateGravity(const double pitch, const double dist, const double bulletSpeed);

	void Process(unsigned char* pImage);	//	接口

private:
	ossian::Utils::ConfigLoader<Config::ConfigSchema>* m_Config = nullptr;

	ossian::IOData<RobotStatus>* m_RobotStatusListener;
	ossian::IOData<ShootData>* m_ShootDataListener;

	AutoAimStatus m_AutoAimStatus;
	ossian::IOData<AutoAimStatus>* m_AutoAimStatusSender;
	ossian::UARTManager* m_UARTManager;
};