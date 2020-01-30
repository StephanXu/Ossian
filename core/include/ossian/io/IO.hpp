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
		virtual FileDescriptor FD() const noexcept = 0;
		virtual std::string Location() const noexcept = 0;
		virtual bool Open() = 0;
		virtual bool Close() = 0;
		virtual bool AddReceiveCallback(unsigned int id, std::function<ReceiveCallback> callback) = 0;
		virtual bool RemoveReceiveCallback(unsigned int id) = 0;
		virtual FrameData Read() = 0; // 带回调的读函数
		virtual void WriteRaw(unsigned int id, size_t length, uint8_t* data) = 0;
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
		virtual std::shared_ptr<IIO> AddDevice(std::string location) = 0;
		// 销毁设备
		virtual bool DelDevice(std::string location) = 0;
		virtual bool DelDevice(FileDescriptor fd) = 0;
		virtual bool DelDevice(std::shared_ptr<IIO> dev) = 0;
		// 添加回调
		virtual bool AddCallback(std::string location, unsigned int id, std::function<ReceiveCallback> callback) = 0;
		virtual bool AddCallback(FileDescriptor fd, uint32_t id, std::function<ReceiveCallback> callback) = 0;
		virtual bool AddCallback(std::shared_ptr<IIO> dev, uint32_t id, std::function<ReceiveCallback> callback) = 0;
		// 获取设备
		virtual std::shared_ptr<IIO> FindDevice(std::string location) = 0;
		virtual std::shared_ptr<IIO> FindDevice(FileDescriptor fd) = 0;
		
		virtual std::vector<FileDescriptor> FDs() = 0;
	};
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO