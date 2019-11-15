#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#ifdef _WIN32
#include <Windows.h>
#endif

#include <string>

namespace NautilusVision
{
namespace IO
{
#ifdef _WIN32

class SerialPort
{
public:

	/**
	 * @enum	Parity
	 *
	 * @brief	У�鷽ʽ�����ڹ��캯����Open������
	 */
	enum Parity
	{
		NoParity = 0, //< ��У��
		OddParity = 1, //< ��У��
		EvenParity = 2, //< żУ��
		MarkParity = 3 //< ���У��
	};

	/**
	 * @enum	StopBit
	 *
	 * @brief	ֹͣλ�����ڹ��캯����Open������
	 */
	enum StopBit
	{
		OneStopBit = 0, //< 1λֹͣλ
		One5StopBits = 1, //< 1.5λֹͣλ
		TwoStopBits = 2 //< 2λֹͣλ
	};

	/**
	 * @fn	SerialPort::SerialPort(std::string portname, unsigned int baudrate, Parity parity, unsigned char databit, StopBit stopbit, bool synchronizeFlag = 1)
	 *
	 * @brief	Constructor
	 *
	 * @param	portname	   	portname(������): ��Windows����"COM1""COM2"�ȣ���Linux����"/dev/ttyS1"��.
	 * @param	baudrate	   	baudrate(������): 9600��19200��38400��43000��56000��57600��115200.
	 * @param	parity		   	parity(У��λ): 0Ϊ��У�飬1Ϊ��У�飬2ΪżУ�飬3Ϊ���У�飨��������windows)
	 * @param	databit		   	databit(����λ): 4-8(windows),5-8(linux)��ͨ��Ϊ8λ.
	 * @param	stopbit		   	stopbit(ֹͣλ): 1Ϊ1λֹͣλ��2Ϊ2λֹͣλ,3Ϊ1.5λֹͣλ.
	 * @param	synchronizeFlag	(Optional) synchronizeflag(ͬ�����첽,��������windows): 0Ϊ�첽��1Ϊͬ��
	 */
	SerialPort(std::string portname,
			   unsigned int baudrate,
			   Parity parity,
			   unsigned char databit,
			   StopBit stopbit,
			   bool synchronizeFlag = 1);

	SerialPort();
	~SerialPort();

	/**
	 * @fn	bool SerialPort::Open(std::string portname, unsigned int baudrate, Parity parity, unsigned char databit, StopBit stopbit, bool synchronizeFlag = 1)
	 *
	 * @brief	@Brief:		�򿪴���,�ɹ�����true��ʧ�ܷ���false
	 *
	 * @exception	std::runtime_error	Raised when a runtime error condition occurs.
	 *
	 * @param	portname	   	portname(������): ��Windows����"COM1""COM2"�ȣ���Linux����"/dev/ttyS1"��.
	 * @param	baudrate	   	baudrate(������): 9600��19200��38400��43000��56000��57600��115200.
	 * @param	parity		   	parity(У��λ): 0Ϊ��У�飬1Ϊ��У�飬2ΪżУ�飬3Ϊ���У�飨��������windows)
	 * @param	databit		   	databit(����λ): 4-8(windows),5-8(linux)��ͨ��Ϊ8λ.
	 * @param	stopbit		   	stopbit(ֹͣλ): 1Ϊ1λֹͣλ��2Ϊ2λֹͣλ,3Ϊ1.5λֹͣλ.
	 * @param	synchronizeFlag	(Optional) synchronizeflag(ͬ�����첽,��������windows): 0Ϊ�첽��1Ϊͬ��
	 *
	 * @returns	True if it succeeds, false if it fails.
	 */
	bool Open(std::string portname,
			  unsigned int baudrate,
			  Parity parity,
			  unsigned char databit,
			  StopBit stopbit,
			  bool synchronizeFlag = 1);


	/**
	 * @fn	unsigned int SerialPort::Send(const unsigned char* buf, int len)
	 *
	 * @brief	�������ݻ�д���ݣ��ɹ����ط������ݳ��ȣ�ʧ�ܷ���0
	 *
	 * @exception	std::runtime_error	Raised when a runtime error condition occurs.
	 *
	 * @param	buf	The buffer.
	 * @param	len	The length.
	 *
	 * @returns	����ʱ���سɹ�д����ֽ�����ͬ�������Ƿ�ɹ����첽)
	 */
	unsigned int Send(const unsigned char* buf, int len);

	/**
	 * @fn	unsigned int SerialPort::Receive(unsigned char* buf, int maxlen)
	 *
	 * @brief	�������ݻ�����ݣ��ɹ����ض�ȡʵ�����ݵĳ��ȣ�ʧ�ܷ���0
	 *
	 * @param [in,out]	buf   	If non-null, the buffer.
	 * @param 		  	maxlen	The maxlen.
	 *
	 * @returns	An int.
	 */
	unsigned int Receive(unsigned char* buf, int maxlen);

	/**
	 * @fn	void SerialPort::Close()
	 *
	 * @brief	�رմ���
	 *
	 */
	void Close();

	/**
	 * @fn	bool SerialPort::IsSync();
	 *
	 * @brief	Query if this object is synchronise
	 *
	 * @returns	True if synchronise, false if not.
	 */
	bool IsSync();

	/**
	 * @fn	bool SerialPort::IsOpened();
	 *
	 * @brief	Query if this serial port is opened
	 *
	 * @returns	True if opened, false if not.
	 */
	bool IsOpened();
private:
	bool m_IsOpened = false;
	bool m_IsSync = true;
	HANDLE m_Handle = NULL;
};

#endif //_WIN32

} //IO

} //NautilusVision

#endif //SERIALPORT_HPP