/**
 * @file	core\include\ossian\io\IO.hpp.
 *
 * @brief	IO类的基础定义
 */

#ifndef OSSIAN_CORE_IO
#define OSSIAN_CORE_IO
#ifdef __linux__
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>

#include "IOTypes.hpp"

/**
 * @namespace	ossian
 *
 * @brief	.
 */

namespace ossian {

	/**
	 * @class	BaseDevice
	 *
	 * @brief	A base device.
	 *
	 * @author	Mlekow
	 * @date	2020/1/31
	 */

	class BaseDevice
	{
	public:

		/**
		 * @fn	virtual std::shared_ptr<IIOBus> BaseDevice::Bus() const noexcept = 0;
		 *
		 * @brief	Gets the bus
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	A std::shared_ptr&lt;IIOBus&gt;
		 */

		virtual std::shared_ptr<IIOBus> Bus() const noexcept = 0;

		/**
		 * @fn	virtual void BaseDevice::WriteRaw(size_t length, uint8_t* data) = 0;
		 *
		 * @brief	Writes a raw
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 		  	length	The length.
		 * @param [in,out]	data  	If non-null, the data.
		 */

		virtual void WriteRaw(size_t length, uint8_t* data) = 0;

		/**
		 * @fn	virtual void BaseDevice::Invoke(size_t length, uint8_t* data) = 0;
		 *
		 * @brief	Executes the given operation on a different thread, and waits for the result
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 		  	length	The length.
		 * @param [in,out]	data  	If non-null, the data.
		 */

		virtual void Invoke(size_t length, uint8_t* data) = 0;
		virtual void ResetCallback(std::function<ReceiveCallback> callback) = 0;
	protected:
		/** @brief	回调函数 */
		std::function<ReceiveCallback> callback;
	};

	/**
	 * @class	IIOBus
	 *
	 * @brief	An iio bus.
	 *
	 * @author	Mlekow
	 * @date	2020/1/31
	 */

	class IIOBus
	{
	public:

		/**
		 * @fn	virtual FileDescriptor IIOBus::FD() const = 0;
		 *
		 * @brief	Gets the fd
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	A FileDescriptor.
		 */

		virtual FileDescriptor FD() const = 0;

		/**
		 * @fn	virtual std::string IIOBus::Location() const noexcept = 0;
		 *
		 * @brief	Gets the location
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	A std::string.
		 */

		virtual std::string Location() const noexcept = 0;

		/**
		 * @fn	virtual bool IIOBus::Open() = 0;
		 *
		 * @brief	Opens this object
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	True if it succeeds, false if it fails.
		 */

		virtual bool Open() = 0;

		/**
		 * @fn	virtual bool IIOBus::Close() = 0;
		 *
		 * @brief	Closes this object
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	True if it succeeds, false if it fails.
		 */

		virtual bool Close() = 0;

		/**
		 * @fn	virtual std::shared_ptr<BaseDevice> IIOBus::AddDevice(unsigned int id, std::function<ReceiveCallback> callback) = 0;
		 *
		 * @brief	Adds a device to 'callback'
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	id			The identifier.
		 * @param 	callback	The callback.
		 *
		 * @returns	A std::shared_ptr&lt;BaseDevice&gt;
		 */

		virtual std::shared_ptr<BaseDevice> AddDevice(unsigned int id, std::function<ReceiveCallback> callback) = 0;

		/**
		 * @fn	virtual bool IIOBus::DelDevice(std::shared_ptr<BaseDevice>) = 0;
		 *
		 * @brief	Deletes the device described by parameter1
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	parameter1	The first parameter.
		 *
		 * @returns	True if it succeeds, false if it fails.
		 */

		virtual bool DelDevice(std::shared_ptr<BaseDevice>) = 0;

		/**
		 * @fn	virtual std::shared_ptr<BaseDevice> IIOBus::Device(unsigned int id) = 0;
		 *
		 * @brief	Devices the given identifier
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	id	The identifier.
		 *
		 * @returns	A std::shared_ptr&lt;BaseDevice&gt;
		 */

		virtual std::shared_ptr<BaseDevice> Device(unsigned int id) = 0;

		/**
		 * @fn	virtual void IIOBus::Read() = 0;
		 *
		 * @brief	Reads this object
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 */

		virtual void Read() = 0; // 带回调的读函数

		/**
		 * @fn	virtual bool IIOBus::IsOpened() = 0;
		 *
		 * @brief	Query if this object is opened
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	True if opened, false if not.
		 */

		virtual bool IsOpened() = 0;
	};

	/**
	 * @class	IIOManager
	 *
	 * @brief	Manager for iioes.
	 *
	 * @author	Mlekow
	 * @date	2020/1/31
	 */

	class IIOManager
	{
	public:

		/**
		 * @fn	virtual IOType IIOManager::Type() const noexcept = 0;
		 *
		 * @brief	Gets the type
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	An IOType.
		 */

		virtual IOType Type() const noexcept = 0;
		// 发送原始数据到指定位置

		/**
		 * @fn	virtual void IIOManager::SendToRaw(std::string location, unsigned int id, size_t length, uint8_t* data) = 0;
		 *
		 * @brief	Sends to raw
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 		  	location	The location.
		 * @param 		  	id			The identifier.
		 * @param 		  	length  	The length.
		 * @param [in,out]	data		If non-null, the data.
		 */

		virtual void SendToRaw(std::string location, unsigned int id, size_t length, uint8_t* data) = 0;
		// 注册设备

		/**
		 * @fn	virtual std::shared_ptr<IIOBus> IIOManager::AddBus(std::string location) = 0;
		 *
		 * @brief	Adds the bus
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	location	The location.
		 *
		 * @returns	A std::shared_ptr&lt;IIOBus&gt;
		 */

		virtual std::shared_ptr<IIOBus> AddBus(std::string location) = 0;
		// 销毁设备

		/**
		 * @fn	virtual bool IIOManager::DelBus(std::string location) = 0;
		 *
		 * @brief	Deletes the bus described by location
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	location	The location.
		 *
		 * @returns	True if it succeeds, false if it fails.
		 */

		virtual bool DelBus(std::string location) = 0;

		/**
		 * @fn	virtual bool IIOManager::DelBus(FileDescriptor fd) = 0;
		 *
		 * @brief	Deletes the bus described by fd
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	fd	The fd.
		 *
		 * @returns	True if it succeeds, false if it fails.
		 */

		virtual bool DelBus(FileDescriptor fd) = 0;

		/**
		 * @fn	virtual bool IIOManager::DelBus(std::shared_ptr<IIOBus> dev) = 0;
		 *
		 * @brief	Deletes the bus described by dev
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	dev	The development.
		 *
		 * @returns	True if it succeeds, false if it fails.
		 */

		virtual bool DelBus(std::shared_ptr<IIOBus> dev) = 0;
		// 获取设备

		/**
		 * @fn	virtual std::shared_ptr<IIOBus> IIOManager::Bus(std::string location) = 0;
		 *
		 * @brief	Bus the given location
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	location	The location.
		 *
		 * @returns	A std::shared_ptr&lt;IIOBus&gt;
		 */

		virtual std::shared_ptr<IIOBus> Bus(std::string location) = 0;

		/**
		 * @fn	virtual std::shared_ptr<IIOBus> IIOManager::Bus(FileDescriptor fd) = 0;
		 *
		 * @brief	Bus the given fd
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @param 	fd	The fd.
		 *
		 * @returns	A std::shared_ptr&lt;IIOBus&gt;
		 */

		virtual std::shared_ptr<IIOBus> Bus(FileDescriptor fd) = 0;

		/**
		 * @fn	virtual std::vector<FileDescriptor> IIOManager::FDs() = 0;
		 *
		 * @brief	Gets the ds
		 *
		 * @author	Mlekow
		 * @date	2020/1/31
		 *
		 * @returns	A std::vector&lt;FileDescriptor&gt;
		 */

		virtual std::vector<FileDescriptor> FDs() = 0;
	};
} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO