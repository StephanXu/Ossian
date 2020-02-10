/**
 * @file	ossian\io\IO.hpp.
 *
 * @brief	IO接口的定义
 */

#ifndef OSSIAN_CORE_IO_INTERFACES
#define OSSIAN_CORE_IO_INTERFACES

#ifdef __linux__
#include <vector>
#include <string>
#include <tuple>
#include <memory>
#include <functional>

 /**
  * @namespace	ossian
  * @brief	.
  */

namespace ossian
{
class BaseDevice;
class IIOBus;
class IIOManager;

using ReceiveCallback = void(
	std::shared_ptr<BaseDevice> device,
	size_t length,
	std::shared_ptr<uint8_t[]> data);

using FileDescriptor = int;
using FrameData = std::tuple<size_t, std::shared_ptr<uint8_t[]>>;

enum class IOType
{
	Undefined,
	UART,
	CAN
};

/**
 * @class	BaseDevice
 * @brief	Device基类
 * @author	Mlekow
 * @date	2020/1/31
 */

class BaseDevice
{
public:
	virtual ~BaseDevice();
	/**
	 * @fn	virtual std::shared_ptr<IIOBus> BaseDevice::Bus() const noexcept = 0;
	 * @brief	获取设备所在总线的句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */

	virtual std::shared_ptr<IIOBus> Bus() = 0;

	/**
	 * @fn	virtual void BaseDevice::WriteRaw(size_t length, uint8_t* data) = 0;
	 * @brief	向总线发送数据
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 		  	length	数据长度
	 * @param [in]		data  	原始数据
	 */

	virtual void WriteRaw(size_t length, std::shared_ptr<uint8_t[]> data) = 0;

	/**
	 * @fn	virtual void BaseDevice::Invoke(size_t length, uint8_t* data) = 0;
	 * @brief	处理回调
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 		  	length	The length.
	 * @param [in,out]	data  	If non-null, the data.
	 */

	virtual void Invoke(size_t length, std::shared_ptr<uint8_t[]> data) = 0;

	/**
	 * @fn	virtual void BaseDevice::SetCallback(std::function<ReceiveCallback> callback) = 0;
	 * @brief	设置设备回调
	 * @author	Mlekow
	 * @date	2020/2/2
	 * @param 	callback	The callback.
	 */

	virtual void SetCallback(std::function<ReceiveCallback> callback) = 0;
};
inline BaseDevice::~BaseDevice() = default;

/**
 * @class	IIOBus
 * @brief	IO总线接口
 * @author	Mlekow
 * @date	2020/1/31
 */

class IIOBus
{
public:
	virtual ~IIOBus();
	/**
	 * @fn	virtual std::shared_ptr<IIOBus> BaseDevice::Bus() const noexcept = 0;
	 * @brief	获取总线管理器的句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */

	virtual std::shared_ptr<IIOManager> Manager() = 0;

	/**
	 * @fn	virtual FileDescriptor IIOBus::FD() const = 0;
	 * @brief	获取总线文件描述符
	 * @author	Mlekow
	 * @date	2020/1/31
	 *
	 * @returns	A FileDescriptor.
	 */

	virtual FileDescriptor FD() const = 0;

	/**
	 * @fn	virtual std::string IIOBus::Location() const noexcept = 0;
	 * @brief	获取总线Location
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::string.
	 */

	virtual std::string Location() const noexcept = 0;


	/**
	 * @fn	virtual bool IIOBus::IsOpened() = 0;
	 * @brief	查询总线是否开启
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	True if opened, false if not.
	 */

	virtual bool IsOpened() const noexcept = 0;

	/**
	 * @fn	virtual bool IIOBus::Open() = 0;
	 * @brief	打开总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	True if it succeeds, false if it fails.
	 */

	virtual bool Open() = 0;

	/**
	 * @fn	virtual bool IIOBus::Close() = 0;
	 * @brief	关闭总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	True if it succeeds, false if it fails.
	 */

	virtual bool Close() = 0;

	/**
	 * @fn	virtual void IIOBus::Read() = 0;
	 * @brief	读取数据、触发对应的回调，注意此处并不会直接返回读到的数据
	 * @author	Mlekow
	 * @date	2020/1/31
	 */

	virtual void Read() = 0;

	/**
	 * @fn	virtual std::vector<std::shared_ptr<BaseDevice>> IIOBus::GetDevices() = 0;
	 * @brief	获取所申请的所有Device
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::vector&lt;FileDescriptor&gt;
	 */

	virtual std::vector<std::shared_ptr<BaseDevice>> GetDevices() = 0;
};
inline IIOBus::~IIOBus() = default;

/**
 * @class	IIOManager
 * @brief	IO总线的管理器
 * @author	Mlekow
 * @date	2020/1/31
 */

class IIOManager
{
public:
	virtual ~IIOManager();
	/**
	 * @fn	virtual IOType IIOManager::Type() const noexcept = 0;
	 * @brief	所管理的IO类型
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	An IOType.
	 */

	virtual IOType Type() const noexcept = 0;

	// 注册设备

	/**
	 * @fn	virtual std::shared_ptr<IIOBus> IIOManager::AddBus(std::string location) = 0;
	 * @brief	添加Bus，返回具体句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */

	virtual std::shared_ptr<IIOBus> AddBus(std::string location) = 0;
	// 销毁设备

	/**
	 * @fn	virtual bool IIOManager::DelBus(std::string location) = 0;
	 * @brief	通过Location删除Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	True if it succeeds, false if it fails.
	 */

	virtual bool DelBus(std::string location) = 0;

	/**
	 * @fn	virtual bool IIOManager::DelBus(std::shared_ptr<IIOBus> dev) = 0;
	 * @brief	通过句柄删除Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	bus	总线的句柄
	 * @returns	True if it succeeds, false if it fails.
	 */

	virtual bool DelBus(std::shared_ptr<IIOBus> bus) = 0;

	/**
	 * @fn	virtual std::shared_ptr<IIOBus> IIOManager::Bus(std::string location) = 0;
	 * @brief	通过指定的Location去获取Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */
	virtual std::shared_ptr<IIOBus> Bus(std::string location) = 0;

	/**
	 * @fn	virtual bool IIOManager::WriteTo(std::shared_ptr<BaseDevice> dev) = 0;
	 * @brief	写入具体的设备
	 * @author	Mlekow
	 * @date	2020/2/2
	 * @param 	device	设备的句柄
	 * @returns	True if it succeeds, false if it fails.
	 */

	virtual void WriteTo(std::shared_ptr<BaseDevice> device, size_t length, std::shared_ptr<uint8_t[]> data) = 0;

	/**
	 * @fn	virtual std::vector<IIOBus> IIOManager::GetBuses() = 0;
	 * @brief	获取所管理的所有总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::vector&lt;FileDescriptor&gt;
	 */

	virtual std::vector<std::shared_ptr<IIOBus>> GetBuses() = 0;
};
inline IIOManager::~IIOManager() = default;
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO_INTERFACES