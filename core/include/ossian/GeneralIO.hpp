#ifndef OSSIAN_CORE_GENERAL_IO
#define OSSIAN_CORE_GENERAL_IO

#include "Factory.hpp"
#include "IOData.hpp"
#include "io/IO.hpp"
#include "ApplicationBuilder.hpp"

#include <mutex>
#include <memory>

namespace ossian
{
//template <typename ModelType,
//          typename IOManagerType,
//          typename Mutex>
//class GeneralIOServiceBuilder
//{
//	using ServiceType = GeneralIO<ModelType, IOManagerType, Mutex>;
//	using BaseBuilder = BaseServiceBuilder<ServiceType>;
//	ApplicationBuilder& m_AppBuilder;
//	BaseBuilder& m_BaseBuilder;
//public:
//	GeneralIOServiceBuilder(ApplicationBuilder& appBuilder, BaseBuilder& config)
//		: m_AppBuilder(appBuilder), m_BaseBuilder(config)
//	{
//	}
//
//	template <typename ... Args>
//	auto Add(Args ...args) -> GeneralIOServiceBuilder&
//	{
//		m_BaseBuilder.AddConfig(
//			[args...](ServiceType& option)
//			{
//				option.Add(args...);
//			});
//		return *this;
//	}
//};


class GeneralIOParseFailed : public std::runtime_error
{
public:
	GeneralIOParseFailed(std::string message) : std::runtime_error(message)
	{
	}
};

template <typename ModelType,
          typename IOManagerType,
          typename Mutex=std::mutex>
class GeneralIO : public IODataBuilder<Mutex, ModelType>
{
	IOManagerType* m_IOManager;
	IOData<ModelType>* m_IOData;
	Mutex m_Mutex;

public:
	OSSIAN_SERVICE_SETUP(GeneralIO(IOManagerType* ioManager, IOData<ModelType>* ioData))
		: m_IOManager(ioManager)
		  , m_IOData(ioData)
	{
	}

	virtual ~GeneralIO() = default;

	template <typename ... Args>
	auto Add(Args ...args) -> void
	{
		m_IOManager->AddDevice(args...)->SetCallback(
			[this](const std::shared_ptr<BaseDevice>& device,
			       const size_t length,
			       const uint8_t* data)
			{
				try
				{
					ModelType model;
					ModelType::Parse(model, data, length);
					m_IOData->Set(model);
				}
				catch (GeneralIOParseFailed& err)
				{
					SPDLOG_WARN("GeneralIOParseFailed: {}", err.what());
				}
			});
	}
};
} // ossian

#define OSSIAN_EXPORT_GENERAL_IO(IOType, StatusType, Method) \
	template <typename Mutex = std::mutex> \
	using IOType = ossian::GeneralIO<StatusType, Method, Mutex>; \
	using IOType##Mt = IOType<std::mutex>; \
	using IOType##St = IOType<ossian::null_mutex>;

#define OSSIAN_EXPORT_GENERAL_CAN_IO(IOType, StatusType) \
	OSSIAN_EXPORT_GENERAL_IO(IOType, StatusType, ossian::CANManager)
#define OSSIAN_EXPORT_GENERAL_UART_IO(IOType, StatusType) \
	OSSIAN_EXPORT_GENERAL_IO(IOType, StatusType, ossian::UARTManager)

#endif // OSSIAN_CORE_GENERAL_IO
