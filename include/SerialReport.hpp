
#ifndef SERIAL_REPORT_HPP
#define SERIAL_REPORT_HPP

#include <nv/nv.hpp>
#include <opencv2/opencv.hpp>

#include "InputAdapter.hpp"

#include <vector>
#include <tuple>
#include <queue>
#include <atomic>
#include <memory>

namespace Ioap = NautilusVision::IOAP;

/**
 * @brief 大风车检测
 *
 * 识别、计算和预测大风车轨迹
 *
 */
class SerialReport : public Ioap::IExecutable
{
public:
	static std::unique_ptr<SerialReport> CreateSerialReport(SerialPortIO* serialPort)
	{
		return std::unique_ptr<SerialReport>(new SerialReport(serialPort));
	}

	/**
	 * 计算一张图像Process an image to refresh Targets, Center and Radius.
	 * 计算一张图像并刷新（当达到最大采样数量时）当前目标坐标、运动圆心和半径信息
	 * @param input 输入数据指针
	 */
	void Process(Ioap::BaseInputData* input) override
	{
		SerialPortIO::InModel model;
		m_SerialPort->Intercept(model);
		spdlog::info("gimbalIndex: {}\tyaw: {}\tpitch: {}", model.gimbalIndex, model.yawMotor, model.pitchMotor);
	}

	bool IsSkip(const Ioap::BaseStatus& refStatus) override
	{
		return m_Valid;
	}

private:
	/**
	 * 构造并初始化一部分参数
	 * @param sampleNum 最大采样数量
	 * @param colorFilter IFilter 类型的颜色过滤器.
	 */
	explicit SerialReport(SerialPortIO* serialPort)
		:m_SerialPort(serialPort)
	{
	}

	std::atomic_bool m_Valid;
	SerialPortIO* m_SerialPort;
};

#endif //SERIAL_REPORT_HPP