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
 * @class	IIOBus
 * @brief	IO总线接口
 * @author	Mlekow
 * @date	2020/2/14
 */
class IIOBus;


/**
 * @class	IIOManager
 * @brief	IO管理器接口
 * @author	Mlekow
 * @date	2020/2/14
 */
class IIOManager;

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
	 * @fn	virtual const std::shared_ptr<IIOBus> BaseDevice::Bus() const = 0;
	 * @brief	获取设备所在总线的句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */
	virtual const std::shared_ptr<IIOBus> Bus() const = 0;


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
 * @class	IIOBus
 *
 * @brief	IO总线接口
 *
 * @author	Mlekow
 * @date	2020/1/31
 */
class IIOBus
{
public:
	/**
	 * @fn	virtual IIOBus::~IIOBus();
	 * @brief	Destructor
	 * @author	Mlekow
	 * @date	2020/2/14
	 */
	virtual ~IIOBus();


	/**
	 * @fn	virtual const std::shared_ptr<IIOManager> IIOBus::Manager() = 0;
	 * @brief	获取总线管理器的句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */
	virtual const std::shared_ptr<IIOManager> Manager() = 0;


	/**
	 * @fn	virtual FileDescriptor IIOBus::FD() const = 0;
	 * @brief	获取总线文件描述符
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A FileDescriptor.
	 */
	virtual FileDescriptor FD() const = 0;


	/**
	 * @fn	virtual const std::string IIOBus::Location() const noexcept = 0;
	 * @brief	获取总线Location
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::string.
	 */
	virtual const std::string Location() const noexcept = 0;


	/**
	 * @fn	virtual bool IIOBus::IsOpened() const noexcept = 0;
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


/**
 * @fn	inline IIOBus::~IIOBus() = default;
 * @brief	Destructor
 * @author	Mlekow
 * @date	2020/2/14
 */
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
	/**
	 * @fn	virtual IIOManager::~IIOManager();
	 * @brief	Destructor
	 * @author	Mlekow
	 * @date	2020/2/14
	 */
	virtual ~IIOManager();


	/**
	 * @fn	virtual IOType IIOManager::Type() const noexcept = 0;
	 * @brief	所管理的IO类型
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	An IOType.
	 */
	virtual IOType Type() const noexcept = 0;


	/**
	 * @fn	virtual const std::shared_ptr<IIOBus> IIOManager::AddBus(std::string const& location) = 0;
	 * @brief	添加Bus，返回具体句柄
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */
	virtual const std::shared_ptr<IIOBus> AddBus(std::string const& location) = 0;


	/**
	 * @fn	virtual bool IIOManager::DelBus(std::string const& location) = 0;
	 * @brief	通过Location删除Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	True if it succeeds, false if it fails.
	 */
	virtual bool DelBus(std::string const& location) = 0;


	/**
	 * @fn	virtual bool IIOManager::DelBus(std::shared_ptr<IIOBus> bus) = 0;
	 * @brief	通过句柄删除Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	bus	总线的句柄.
	 * @returns	True if it succeeds, false if it fails.
	 */
	virtual bool DelBus(std::shared_ptr<IIOBus> bus) = 0;


	/**
	 * @fn	virtual const std::shared_ptr<IIOBus> IIOManager::Bus(std::string const& location) const = 0;
	 * @brief	通过指定的Location去获取Bus
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	location	The location.
	 * @returns	A std::shared_ptr&lt;IIOBus&gt;
	 */
	virtual const std::shared_ptr<IIOBus> Bus(std::string const& location) const = 0;


	/**
	 * @fn	virtual void IIOManager::WriteTo(std::shared_ptr<BaseDevice> const& device, const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;
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
	 * @fn	virtual const std::vector<std::shared_ptr<IIOBus>> IIOManager::GetBuses() const = 0;
	 * @brief	获取所管理的所有总线
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @returns	A std::vector&lt;FileDescriptor&gt;
	 */
	virtual const std::vector<std::shared_ptr<IIOBus>> GetBuses() const = 0;
};

inline IIOManager::~IIOManager() = default;
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO_INTERFACES