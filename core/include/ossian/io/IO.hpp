#ifndef OSSIAN_CORE_IO
#define OSSIAN_CORE_IO
#ifdef __linux__
#include <vector>
#include <string>
#include <unordered_map>
#include "IOTypes.hpp"
#include "IODeviceMap.hpp"
namespace ossian {
	class IIO
	{
	public:
		FileDescriptor FD() const noexcept { return m_FD; }
		std::string Location() const noexcept { return m_Location; }
		virtual bool Open() = 0;
		virtual bool Close() = 0;
		virtual bool AddReceiveCallback(unsigned int id, std::function<ReceiveCallback> callback) = 0;
		virtual bool RemoveReceiveCallback(unsigned int id) = 0;
		virtual FrameData Read() = 0; // 带回调的读函数
		virtual void WriteRaw(unsigned int id, size_t length, uint8_t* data) = 0;
	protected:
		FileDescriptor m_FD;
		std::string m_Location;
		std::unordered_map<unsigned int, std::function<ReceiveCallback>> m_InIdMap;
	};

	class IIOManager
	{
	public:
		virtual IOType Type() const noexcept = 0;
		// 发送原始数据到指定位置
		virtual void SendToRaw(std::string location, unsigned int id, size_t length, uint8_t* data) = 0;
		// 从指定位置读取FrameData并触发回调
		virtual FrameData ReadFrom(std::string location) = 0;
		// 注册设备
		virtual bool AddDevice(std::string location) = 0;
		// 销毁设备
		virtual bool DelDevice(std::string location) = 0;
		// 获取设备描述符
		virtual FileDescriptor DeviceFD(std::string location) = 0;
		// 添加回调
		virtual void AddCallback(std::string location, unsigned int id, std::function<ReceiveCallback> callback) = 0;
		// 获取设备
		virtual IIO* FindDevice(std::string location) = 0;
		virtual IIO* FindDevice(FileDescriptor fd) = 0;
		virtual std::vector<FileDescriptor> FDs() = 0;
	};
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO