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

namespace ossian
{
/**
 * @class	BaseDevice
 * @brief	设备基类
 * @author	Mlekow
 * @date	2020/2/14
 */
class BaseDevice;


/**
 * @class	BaseHardwareBus
 * @brief	IO总线接口
 * @author	Mlekow
 * @date	2020/2/14
 */
class BaseHardwareBus;


/**
 * @class	BaseHardwareManager
 * @brief	IO管理器接口
 * @author	Mlekow
 * @date	2020/2/14
 */
class BaseHardwareManager;

/** @brief	The receive callback */
using ReceiveCallback = void(
	std::shared_ptr<BaseDevice> device,
	size_t length,
	std::shared_ptr<uint8_t[]> data);

/** @brief	Information describing the file */
using FileDescriptor = int;
/** @brief	Information describing the frame */
using FrameData = std::tuple<size_t, std::shared_ptr<uint8_t[]>>;


/**
 * @enum	IOType
 *
 * @brief	Values that represent I/O types
 */
enum class IOType
{
	Undefined,
	UART,
	CAN
};


/**
 * @class	BaseDevice
 *
 * @brief	Device基类
 *
 * @author	Mlekow
 * @date	2020/1/31
 */
class BaseDevice
{
public:
	/**
	 * @fn	virtual BaseDevice::~BaseDevice();
	 * @brief	Destructor
	 * @author	Mlekow
	 * @date	2020/2/14
	 */
	virtual ~BaseDevice();


	/**
	 * @fn	virtual const std::shared_ptr<BaseHardwareBus> BaseDevice::Bus() const = 0;
	 * @brief	获取设备所在总线的句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::shared_ptr&lt;BaseHardwareBus&gt;
	 */
	virtual const std::shared_ptr<BaseHardwareBus> Bus() const = 0;


	/**
	 * @fn	virtual void BaseDevice::WriteRaw(const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;
	 * @brief	向总线发送数据
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	  	length	数据长度.
	 * @param [in]	data  	原始数据.
	 */
	virtual void WriteRaw(const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;


	/**
	 * @fn	virtual void BaseDevice::Invoke(const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;
	 * @brief	处理回调
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	length	The length.
	 * @param 	data  	If non-null, the data.
	 */
	virtual void Invoke(const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;


	/**
	 * @fn	virtual void BaseDevice::SetCallback(std::function<ReceiveCallback> const& callback) = 0;
	 * @brief	设置设备回调
	 * @author	Mlekow
	 * @date	2020/2/2
	 * @param 	callback	The callback.
	 */
	virtual void SetCallback(std::function<ReceiveCallback> const& callback) = 0;
};

inline BaseDevice::~BaseDevice() = default;


/**
 * @class	BaseHardwareBus
 *
 * @brief	IO总线接口
 *
 * @author	Mlekow
 * @date	2020/1/31
 */
class BaseHardwareBus
{
public:
	/**
	 * @fn	virtual BaseHardwareBus::~BaseHardwareBus();
	 * @brief	Destructor
	 * @author	Mlekow
	 * @date	2020/2/14
	 */
	virtual ~BaseHardwareBus();


	/**
	 * @fn	virtual const std::shared_ptr<BaseHardwareManager> BaseHardwareBus::Manager() = 0;
	 * @brief	获取总线管理器的句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::shared_ptr&lt;BaseHardwareBus&gt;
	 */
	virtual const std::shared_ptr<BaseHardwareManager> Manager() = 0;


	/**
	 * @fn	virtual FileDescriptor BaseHardwareBus::FD() const = 0;
	 * @brief	获取总线文件描述符
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A FileDescriptor.
	 */
	virtual FileDescriptor FD() const = 0;


	/**
	 * @fn	virtual const std::string BaseHardwareBus::Location() const noexcept = 0;
	 * @brief	获取总线Location
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::string.
	 */
	virtual const std::string Location() const noexcept = 0;


	/**
	 * @fn	virtual bool BaseHardwareBus::IsOpened() const noexcept = 0;
	 * @brief	查询总线是否开启
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	True if opened, false if not.
	 */
	virtual bool IsOpened() const noexcept = 0;


	/**
	 * @fn	virtual bool BaseHardwareBus::Open() = 0;
	 * @brief	打开总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	True if it succeeds, false if it fails.
	 */
	virtual bool Open() = 0;


	/**
	 * @fn	virtual bool BaseHardwareBus::Close() = 0;
	 * @brief	关闭总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	True if it succeeds, false if it fails.
	 */
	virtual bool Close() = 0;


	/**
	 * @fn	virtual void BaseHardwareBus::Read() = 0;
	 * @brief	读取数据、触发对应的回调，注意此处并不会直接返回读到的数据
	 * @author	Mlekow
	 * @date	2020/1/31
	 */
	virtual void Read() = 0;


	/**
	 * @fn	virtual std::vector<std::shared_ptr<BaseDevice>> BaseHardwareBus::GetDevices() = 0;
	 * @brief	获取所申请的所有Device
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::vector&lt;FileDescriptor&gt;
	 */
	virtual std::vector<std::shared_ptr<BaseDevice>> GetDevices() = 0;
};


/**
 * @fn	inline BaseHardwareBus::~BaseHardwareBus() = default;
 * @brief	Destructor
 * @author	Mlekow
 * @date	2020/2/14
 */
inline BaseHardwareBus::~BaseHardwareBus() = default;


/**
 * @class	BaseHardwareManager
 * @brief	IO总线的管理器
 * @author	Mlekow
 * @date	2020/1/31
 */
class BaseHardwareManager
{
public:
	/**
	 * @fn	virtual BaseHardwareManager::~BaseHardwareManager();
	 * @brief	Destructor
	 * @author	Mlekow
	 * @date	2020/2/14
	 */
	virtual ~BaseHardwareManager();


	/**
	 * @fn	virtual IOType BaseHardwareManager::Type() const noexcept = 0;
	 * @brief	所管理的IO类型
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	An IOType.
	 */
	virtual IOType Type() const noexcept = 0;


	/**
	 * @fn	virtual const std::shared_ptr<BaseHardwareBus> BaseHardwareManager::Bus(std::string const& location) const = 0;
	 * @brief	通过指定的Location去获取Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	A std::shared_ptr&lt;BaseHardwareBus&gt;
	 */
	virtual const std::shared_ptr<BaseHardwareBus> Bus(std::string const& location) const = 0;


	/**
	 * @fn	virtual void BaseHardwareManager::WriteTo(std::shared_ptr<BaseDevice> const& device, const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;
	 * @brief	写入具体的设备
	 * @author	Mlekow
	 * @date	2020/2/2
	 * @param 	device	设备的句柄.
	 * @param 	length	数据长度.
	 * @param 	data  	数据指针.
	 */
	virtual void WriteTo(std::shared_ptr<BaseDevice> const& device, const size_t length,
		std::shared_ptr<uint8_t[]> const& data) = 0;


	/**
	 * @fn	virtual const std::vector<std::shared_ptr<BaseHardwareBus>> BaseHardwareManager::GetBuses() const = 0;
	 * @brief	获取所管理的所有总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::vector&lt;FileDescriptor&gt;
	 */
	virtual const std::vector<std::shared_ptr<BaseHardwareBus>> GetBuses() const = 0;
};

inline BaseHardwareManager::~BaseHardwareManager() = default;
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO_INTERFACES