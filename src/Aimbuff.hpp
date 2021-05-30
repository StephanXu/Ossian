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
constexpr double PARAM = 30;				//��ǰ���Ƕ�(��λ:��)
constexpr int preparing_time = 240;	// ��ת�������ʱ��
constexpr int MAX_QUEUE_LENGTH = 50;

constexpr double CAM_TO_GIMBAL_Y = -41.80;  // mm
constexpr double DISTANCE_TO_BUFF = 7000; // mm
constexpr double BULLET_SPEED = 29; // m/s

class Aimbuff : public ossian::IODataBuilder<std::mutex, AutoAimStatus>
{
public:
	cv::Mat srcImage;				//Ҫ�����ԭͼ
	cv::Mat binaryImage/*, dstImage*/;	//�м���̵Ķ�ֵͼ(����ȷ���������)�������ڻ��ϱ�ǵ�����ͼ
	
	int myColor = 0;			//װ�װ���ɫ��Ĭ��ֵ:0(����δ֪)����ɫ:0����ɫ:2

	cv::RotatedRect rect;			//Ŀ��е�Rect
	cv::RotatedRect predict_rect;	//Ԥ�����
	bool tag;						//1:�ҵ�����		0:û���ҵ�����	(�������Ҫ��ͼ�ϻ�����Ŀ��еĻ�)
	cv::Point2f center;					//Ŀ����ΰ���(imgprocess�����л��)
	
	std::deque<cv::Point2f> cSet;		//һϵ�а��ĵĴ洢������������С���˷����Բ��
	cv::Point2f Centroid;			//�糵Բ��
	double dRadius;					//�糵Բ�ܰ뾶
	
	cv::Point2f predict_point;			//Ԥ���
	short int rotateDirectionTag = -1;	//�糵��ת����	δ��ת:-1	˳ʱ��:0		��ʱ��:1
	
	//�������������1->4->7->10->13->15
	bool shoot_over = false;		//��������Ƿ����(Ĭ��Ϊfalse)
	unsigned int startRectNum;		//��ʼʱ����������
	unsigned int nextRectNum = 4;	//���ӳ�ʼʱ�̵���һʱ��Ӧ���ֵ���������

	OSSIAN_SERVICE_SETUP(Aimbuff(ossian::Utils::ConfigLoader<Config::ConfigSchema>* config,
		ossian::IOData<AutoAimStatus>* autoAimStatus,
#ifndef VISION_ONLY
		ossian::IOData<RobotStatus>* robotStatus,
		ossian::IOData<ShootData>* shootDataStatus,
#endif // !VISION_ONLY
		ossian::UARTManager* uartManager
		));
	//Aimbuff(cv::Mat _srcImage_);			//���ع��캯������ԭͼ����

	void imgProcess();					//���ͼ�ζ�ֵͼ
	cv::RotatedRect findRect();			//�ҵ�Ŀ���
	bool LeastSquaresCircleFitting();	//��С���˷���ϰ����˶��Ĺ켣Բ
	bool calc(const cv::Point2f& Centroid, const double& dRadius, cv::RotatedRect& curRect);	//������Ԥ�ⷨ
	unsigned int judgeShootNum(cv::Mat &binImg);	//���ݶ�ֵͼ�жϵ�ǰ�򵽵ڼ�����\

	~Aimbuff() {};	//��������

	void drawRect(cv::RotatedRect &rec, cv::Mat dstImage);

	std::tuple<double, double> AngleSolverPinHole(cv::Point2f targetCenter);
	double CompensateOffset(const double pitch, const double dist, const double camToGimbalY);
	double CompensateGravity(const double pitch, const double dist, const double bulletSpeed);

	void Process(unsigned char* pImage);	//	�ӿ�

private:
	ossian::Utils::ConfigLoader<Config::ConfigSchema>* m_Config = nullptr;

	ossian::IOData<RobotStatus>* m_RobotStatusListener;
	ossian::IOData<ShootData>* m_ShootDataListener;

	AutoAimStatus m_AutoAimStatus;
	ossian::IOData<AutoAimStatus>* m_AutoAimStatusSender;
	ossian::UARTManager* m_UARTManager;
};