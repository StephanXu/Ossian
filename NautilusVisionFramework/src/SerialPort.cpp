
#ifdef _WIN32
#include <Windows.h>
#endif

#include <fmt/format.h>

#include <exception>
#include <stdexcept>
#include <string>

#include "nv/SerialPort.hpp"

namespace NautilusVision
{
namespace IO
{
#ifdef _WIN32

SerialPort::SerialPort(std::string portname,
					   unsigned int baudrate,
					   Parity parity,
					   unsigned char databit,
					   StopBit stopbit,
					   bool synchronizeFlag)
{
	Open(portname, baudrate, parity, databit, stopbit, synchronizeFlag);
}

SerialPort::SerialPort()
{
}

SerialPort::~SerialPort()
{
	Close();
}

SerialPort::SerialPort(SerialPort&& serialPort) noexcept
{
	*this = std::move(serialPort);
}

SerialPort& SerialPort::operator=(SerialPort&& rhs) noexcept
{
	m_IsOpened = rhs.m_IsOpened;
	m_IsSync = rhs.m_IsSync;
	m_Handle = rhs.m_Handle;

	rhs.m_IsOpened = false;
	rhs.m_Handle = NULL;
	return *this;
}

bool SerialPort::Open(std::string portname,
					  unsigned int baudrate,
					  Parity parity,
					  unsigned char databit,
					  StopBit stopbit,
					  bool synchronizeFlag)
{
	m_IsSync = synchronizeFlag;
	m_Handle = CreateFileA(portname.c_str(), //串口名
						   GENERIC_READ | GENERIC_WRITE, //支持读写
						   0, //独占方式，串口不支持共享
						   NULL,//安全属性指针，默认值为NULL
						   OPEN_EXISTING, //打开现有的串口文件
						   m_IsSync ? 0 : FILE_FLAG_OVERLAPPED, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
						   NULL); //用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL

	if (INVALID_HANDLE_VALUE == m_Handle)
		throw std::runtime_error(fmt::format("Open serial port fail: {}", GetLastError()));

	//配置缓冲区大小
	if (!SetupComm(m_Handle, 1024, 1024))
	{
		std::string err{ fmt::format("SetupComm fail: {}",GetLastError()) };
		CloseHandle(m_Handle);
		m_Handle = NULL;
		throw std::runtime_error(err);
	}

	// 配置参数
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = baudrate; //< 波特率
	dcb.ByteSize = databit; //< 数据位
	dcb.Parity = parity; //< 校验
	dcb.StopBits = stopbit; //< 结束位
	if (!SetCommState(m_Handle, &dcb))
	{
		std::string err{ fmt::format("SetCommState fail: {}",GetLastError()) };
		CloseHandle(m_Handle);
		m_Handle = NULL;
		throw std::runtime_error(err);
	}

	//超时处理,单位：毫秒
	//总超时＝时间系数×读或写的字符数＋时间常量
	COMMTIMEOUTS TimeOuts;
	TimeOuts.ReadIntervalTimeout = 1000; //读间隔超时
	TimeOuts.ReadTotalTimeoutMultiplier = 500; //读时间系数
	TimeOuts.ReadTotalTimeoutConstant = 5000; //读时间常量
	TimeOuts.WriteTotalTimeoutMultiplier = 500; // 写时间系数
	TimeOuts.WriteTotalTimeoutConstant = 2000; //写时间常量
	if (!SetCommTimeouts(m_Handle, &TimeOuts))
	{
		std::string err{ fmt::format("SetCommTimeouts fail: {}",GetLastError()) };
		CloseHandle(m_Handle);
		m_Handle = NULL;
		throw std::runtime_error(err);
	}

	PurgeComm(m_Handle, PURGE_TXCLEAR | PURGE_RXCLEAR); //清空串口缓冲区
	m_IsOpened = true;
	return true;
}

unsigned int SerialPort::Send(const unsigned char* buf, int len)
{
	if (!m_IsOpened)
		return 0;
	if (!buf)
		throw std::invalid_argument("buf cannot be nullptr");

	if (m_IsSync)
	{
		// 同步方式
		DWORD dwBytesWrite = len; //成功写入的数据字节数
		BOOL bWriteStat = WriteFile(m_Handle, //串口句柄
									buf, //数据首地址
									dwBytesWrite, //要发送的数据字节数
									&dwBytesWrite, //DWORD*，用来接收返回成功发送的数据字节数
									NULL); //NULL为同步发送，OVERLAPPED*为异步发送
		if (!bWriteStat)
		{
			return 0;
		}
		return dwBytesWrite;
	}
	else
	{
		//异步方式
		OVERLAPPED osWrite = { 0 };
		DWORD dwWritten, dwToWrite = len;
		DWORD dwRes;
		BOOL fRes;

		// Create this write operation's OVERLAPPED structure's hEvent.
		osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		if (osWrite.hEvent == NULL)
		{
			throw std::runtime_error(fmt::format("error creating overlapped event handle in send: {}",
												 GetLastError()));
			// error creating overlapped event handle
			return FALSE;
		}

		// Issue write.
		if (!WriteFile(m_Handle, buf, dwToWrite, &dwWritten, &osWrite))
		{
			if (GetLastError() != ERROR_IO_PENDING)
			{
				CloseHandle(osWrite.hEvent);
				fRes = FALSE;
				throw std::runtime_error("WriteFile failed, but isn't delayed.");
				// WriteFile failed, but isn't delayed. Report error and abort.
			}
			else
			{
				// Write is pending.
				dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
			}
			switch (dwRes)
			{
				// OVERLAPPED structure's event has been signaled. 
			case WAIT_OBJECT_0:
				if (!GetOverlappedResult(m_Handle, &osWrite, &dwWritten, FALSE))
					fRes = FALSE;
				else
					// Write operation completed successfully.
					fRes = TRUE;
				break;

			default:
				// An error has occurred in WaitForSingleObject.
				// This usually indicates a problem with the
				// OVERLAPPED structure's event handle.
				CloseHandle(osWrite.hEvent);
				fRes = FALSE;
				throw std::runtime_error("An error has occurred in WaitForSingleObject");
				break;
			}
		}
		else
			// WriteFile completed immediately.
			fRes = TRUE;
		CloseHandle(osWrite.hEvent);
		return fRes;
	}
}

unsigned int SerialPort::Receive(unsigned char* buf, int maxlen)
{
	if (m_IsOpened)
		return 0;
	if (!buf)
		throw std::invalid_argument("buf cannot be nullptr");

	if (m_IsSync)
	{
		//同步方式
		DWORD wCount = maxlen; //成功读取的数据字节数
		BOOL bReadStat = ReadFile(m_Handle, //串口句柄
								  buf, //数据首地址
								  wCount, //要读取的数据最大字节数
								  &wCount, //DWORD*,用来接收返回成功读取的数据字节数
								  NULL); //NULL为同步发送，OVERLAPPED*为异步发送
		if (!bReadStat)
		{
			return 0;
		}
		return wCount;
	}
	else
	{
		//异步方式
		DWORD wCount = maxlen; //<成功读取的数据字节数
		DWORD dwErrorFlags; //<错误标志
		COMSTAT comStat; //<通讯状态
		OVERLAPPED m_osRead; //<异步输入输出结构体

							 //创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
		memset(&m_osRead, 0, sizeof(m_osRead));
		m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, "ReadEvent");
		ClearCommError(m_Handle, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态
		if (!comStat.cbInQue)
			return 0; //如果输入缓冲区字节数为0，则返回false

		BOOL bReadStat = ReadFile(m_Handle, //串口句柄
								  buf, //数据首地址
								  wCount, //要读取的数据最大字节数
								  &wCount, //DWORD*,用来接收返回成功读取的数据字节数
								  &m_osRead); //NULL为同步发送，OVERLAPPED*为异步发送
		if (!bReadStat)
		{
			if (GetLastError() == ERROR_IO_PENDING) //如果串口正在读取中
			{
				//GetOverlappedResult函数的最后一个参数设为TRUE
				//函数会一直等待，直到读操作完成或由于错误而返回
				GetOverlappedResult(m_Handle, &m_osRead, &wCount, TRUE);
			}
			else
			{
				ClearCommError(m_Handle, &dwErrorFlags, &comStat); //清除通讯错误
				CloseHandle(m_osRead.hEvent); //关闭并释放hEvent的内存
				return 0;
			}
		}
		return wCount;
	}
}

void SerialPort::Close()
{
	CloseHandle(m_Handle);
}

bool SerialPort::IsSync()
{
	return m_IsSync;
}

bool SerialPort::IsOpened() 
{ 
	return m_IsOpened;
}
#endif //_WIN32
} // IO

} // NautilusVision