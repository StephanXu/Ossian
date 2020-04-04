/**
 * @file	ossian\io\IO.hpp.
 *
 * @brief	IO接口的定义
 */

#ifndef OSSIAN_CORE_IO_INTERFACES
#define OSSIAN_CORE_IO_INTERFACES

#ifdef __linux__
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

/** @brief	The receive callback */
template<typename DeviceType>
using ReceiveCallback = void(
	std::shared_ptr<DeviceType> const& device,
	const size_t length,
	const uint8_t * data);

/** @brief	Information describing the file */
using FileDescriptor = int;

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
	 * @fn	virtual void BaseDevice::WriteRaw(const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;
	 * @brief	向总线发送数据
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	  	length	数据长度.
	 * @param [in]	data  	原始数据.
	 */
	virtual void WriteRaw(const size_t length, const uint8_t* data) const = 0;


	/**
	 * @fn	virtual void BaseDevice::Invoke(const size_t length, std::shared_ptr<uint8_t[]> const& data) = 0;
	 * @brief	处理回调
	 * @author	Mlekow
	 * @date	2020/1/31
	 * @param 	length	The length.
	 * @param 	data  	If non-null, the data.
	 */
	virtual void Invoke(const size_t length, const uint8_t* data) = 0;
};

inline BaseDevice::~BaseDevice() = default;


class IListenable
{
public:
	virtual ~IListenable();
	virtual void Read() const = 0;
	virtual FileDescriptor FD() const noexcept = 0;
};

inline IListenable::~IListenable() = default;

} // ossian
#endif // __linux__
#endif // OSSIAN_CORE_IO_INTERFACES