#include "..\include\nv\SerialPort.hpp"
#include "..\include\nv\SerialPort.hpp"
#include "..\include\nv\SerialPort.hpp"
#include "..\include\nv\SerialPort.hpp"

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

bool SerialPort::Open(std::string portname,
					  unsigned int baudrate,
					  Parity parity,
					  unsigned char databit,
					  StopBit stopbit,
					  bool synchronizeFlag)
{
	m_IsSync = synchronizeFlag;
	m_Handle = CreateFileA(portname.c_str(), //������
						   GENERIC_READ | GENERIC_WRITE, //֧�ֶ�д
						   0, //��ռ��ʽ�����ڲ�֧�ֹ���
						   NULL,//��ȫ����ָ�룬Ĭ��ֵΪNULL
						   OPEN_EXISTING, //�����еĴ����ļ�
						   m_IsSync ? 0 : FILE_FLAG_OVERLAPPED, //0��ͬ����ʽ��FILE_FLAG_OVERLAPPED���첽��ʽ
						   NULL); //���ڸ����ļ������Ĭ��ֵΪNULL���Դ��ڶ��Ըò���������ΪNULL

	if (INVALID_HANDLE_VALUE == m_Handle)
		throw std::runtime_error(fmt::format("Open serial port fail: {}", GetLastError()));

	//���û�������С
	if (!SetupComm(m_Handle, 1024, 1024))
	{
		std::string err{ fmt::format("SetupComm fail: {}",GetLastError()) };
		CloseHandle(m_Handle);
		m_Handle = NULL;
		throw std::runtime_error(err);
	}

	// ���ò���
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = baudrate; //< ������
	dcb.ByteSize = databit; //< ����λ
	dcb.Parity = parity; //< У��
	dcb.StopBits = stopbit; //< ����λ
	if (!SetCommState(m_Handle, &dcb))
	{
		std::string err{ fmt::format("SetCommState fail: {}",GetLastError()) };
		CloseHandle(m_Handle);
		m_Handle = NULL;
		throw std::runtime_error(err);
	}

	//��ʱ����,��λ������
	//�ܳ�ʱ��ʱ��ϵ��������д���ַ�����ʱ�䳣��
	COMMTIMEOUTS TimeOuts;
	TimeOuts.ReadIntervalTimeout = 1000; //�������ʱ
	TimeOuts.ReadTotalTimeoutMultiplier = 500; //��ʱ��ϵ��
	TimeOuts.ReadTotalTimeoutConstant = 5000; //��ʱ�䳣��
	TimeOuts.WriteTotalTimeoutMultiplier = 500; // дʱ��ϵ��
	TimeOuts.WriteTotalTimeoutConstant = 2000; //дʱ�䳣��
	if (!SetCommTimeouts(m_Handle, &TimeOuts))
	{
		std::string err{ fmt::format("SetCommTimeouts fail: {}",GetLastError()) };
		CloseHandle(m_Handle);
		m_Handle = NULL;
		throw std::runtime_error(err);
	}

	PurgeComm(m_Handle, PURGE_TXCLEAR | PURGE_RXCLEAR); //��մ��ڻ�����
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
		// ͬ����ʽ
		DWORD dwBytesWrite = len; //�ɹ�д��������ֽ���
		BOOL bWriteStat = WriteFile(m_Handle, //���ھ��
									buf, //�����׵�ַ
									dwBytesWrite, //Ҫ���͵������ֽ���
									&dwBytesWrite, //DWORD*���������շ��سɹ����͵������ֽ���
									NULL); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
		if (!bWriteStat)
		{
			return 0;
		}
		return dwBytesWrite;
	}
	else
	{
		//�첽��ʽ
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
		//ͬ����ʽ
		DWORD wCount = maxlen; //�ɹ���ȡ�������ֽ���
		BOOL bReadStat = ReadFile(m_Handle, //���ھ��
								  buf, //�����׵�ַ
								  wCount, //Ҫ��ȡ����������ֽ���
								  &wCount, //DWORD*,�������շ��سɹ���ȡ�������ֽ���
								  NULL); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
		if (!bReadStat)
		{
			return 0;
		}
		return wCount;
	}
	else
	{
		//�첽��ʽ
		DWORD wCount = maxlen; //<�ɹ���ȡ�������ֽ���
		DWORD dwErrorFlags; //<�����־
		COMSTAT comStat; //<ͨѶ״̬
		OVERLAPPED m_osRead; //<�첽��������ṹ��

							 //����һ������OVERLAPPED���¼��������������õ�����ϵͳҪ����ô��
		memset(&m_osRead, 0, sizeof(m_osRead));
		m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, "ReadEvent");
		ClearCommError(m_Handle, &dwErrorFlags, &comStat); //���ͨѶ���󣬻���豸��ǰ״̬
		if (!comStat.cbInQue)
			return 0; //������뻺�����ֽ���Ϊ0���򷵻�false

		BOOL bReadStat = ReadFile(m_Handle, //���ھ��
								  buf, //�����׵�ַ
								  wCount, //Ҫ��ȡ����������ֽ���
								  &wCount, //DWORD*,�������շ��سɹ���ȡ�������ֽ���
								  &m_osRead); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
		if (!bReadStat)
		{
			if (GetLastError() == ERROR_IO_PENDING) //����������ڶ�ȡ��
			{
				//GetOverlappedResult���������һ��������ΪTRUE
				//������һֱ�ȴ���ֱ����������ɻ����ڴ��������
				GetOverlappedResult(m_Handle, &m_osRead, &wCount, TRUE);
			}
			else
			{
				ClearCommError(m_Handle, &dwErrorFlags, &comStat); //���ͨѶ����
				CloseHandle(m_osRead.hEvent); //�رղ��ͷ�hEvent���ڴ�
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