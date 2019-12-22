
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
 * @brief ��糵���
 *
 * ʶ�𡢼����Ԥ���糵�켣
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
	 * ����һ��ͼ��Process an image to refresh Targets, Center and Radius.
	 * ����һ��ͼ��ˢ�£����ﵽ����������ʱ����ǰĿ�����ꡢ�˶�Բ�ĺͰ뾶��Ϣ
	 * @param input ��������ָ��
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
	 * ���첢��ʼ��һ���ֲ���
	 * @param sampleNum ����������
	 * @param colorFilter IFilter ���͵���ɫ������.
	 */
	explicit SerialReport(SerialPortIO* serialPort)
		:m_SerialPort(serialPort)
	{
	}

	std::atomic_bool m_Valid;
	SerialPortIO* m_SerialPort;
};

#endif //SERIAL_REPORT_HPP