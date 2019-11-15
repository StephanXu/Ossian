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
	 * @brief	校验方式（用于构造函数或Open函数）
	 */
	enum Parity
	{
		NoParity = 0, //< 无校验
		OddParity = 1, //< 奇校验
		EvenParity = 2, //< 偶校验
		MarkParity = 3 //< 标记校验
	};

	/**
	 * @enum	StopBit
	 *
	 * @brief	停止位（用于构造函数或Open函数）
	 */
	enum StopBit
	{
		OneStopBit = 0, //< 1位停止位
		One5StopBits = 1, //< 1.5位停止位
		TwoStopBits = 2 //< 2位停止位
	};

	/**
	 * @fn	SerialPort::SerialPort(std::string portname, unsigned int baudrate, Parity parity, unsigned char databit, StopBit stopbit, bool synchronizeFlag = 1)
	 *
	 * @brief	Constructor
	 *
	 * @param	portname	   	portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等.
	 * @param	baudrate	   	baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200.
	 * @param	parity		   	parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验（仅适用于windows)
	 * @param	databit		   	databit(数据位): 4-8(windows),5-8(linux)，通常为8位.
	 * @param	stopbit		   	stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位.
	 * @param	synchronizeFlag	(Optional) synchronizeflag(同步、异步,仅适用与windows): 0为异步，1为同步
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
	 * @brief	@Brief:		打开串口,成功返回true，失败返回false
	 *
	 * @exception	std::runtime_error	Raised when a runtime error condition occurs.
	 *
	 * @param	portname	   	portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等.
	 * @param	baudrate	   	baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200.
	 * @param	parity		   	parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验（仅适用于windows)
	 * @param	databit		   	databit(数据位): 4-8(windows),5-8(linux)，通常为8位.
	 * @param	stopbit		   	stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位.
	 * @param	synchronizeFlag	(Optional) synchronizeflag(同步、异步,仅适用与windows): 0为异步，1为同步
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
	 * @brief	发送数据或写数据，成功返回发送数据长度，失败返回0
	 *
	 * @exception	std::runtime_error	Raised when a runtime error condition occurs.
	 *
	 * @param	buf	The buffer.
	 * @param	len	The length.
	 *
	 * @returns	正常时返回成功写入的字节数（同步）或是否成功（异步)
	 */
	unsigned int Send(const unsigned char* buf, int len);

	/**
	 * @fn	unsigned int SerialPort::Receive(unsigned char* buf, int maxlen)
	 *
	 * @brief	接受数据或读数据，成功返回读取实际数据的长度，失败返回0
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
	 * @brief	关闭串口
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