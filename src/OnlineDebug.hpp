/**
 * @file OnlineDebug.hpp
 * @author Xu Zihan (im.xuzihan@outlook.com)
 * @brief Online debug logic
 * @version 0.1
 * @date 2020-02-23
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_ONLINE_DEBUG_HPP
#define OSSIAN_ONLINE_DEBUG_HPP

#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <signalrclient/hub_connection_builder.h>
#include <ossian/Factory.hpp>
#include <ossian/Configuration.hpp>
#include <spdlog/sinks/basic_file_sink.h>


#include "RPC.hpp"

class SignalRLogger : public signalr::log_writer
{
	std::shared_ptr<spdlog::logger> m_Logger;
public:
	SignalRLogger()
		: m_Logger(spdlog::basic_logger_mt("SignalRLogger", "OnlineDebug.log"))
	{
	}

	// Inherited via log_writer
	void __cdecl write(const std::string& entry) override
	{
		m_Logger->info(entry);
		//SPDLOG_TRACE(entry);
	}
};


/**
 * @brief RPC hub (for server calling client).
 */
class OnlineDebugHub : public BaseHub
{
public:
	OnlineDebugHub() = delete;

	OnlineDebugHub(signalr::hub_connection&& connection);

	static auto ReloadSettings(std::string settings) -> void;
};

/**
 * @brief Log sink.
 * 
 * @tparam Mutex Mutex type.
 */
template <typename Mutex>
class online_logger_sink : public spdlog::sinks::base_sink<Mutex>
{
	OnlineDebugHub& m_Hub;
	std::vector<std::string> m_Logs;
	std::string m_LogId;
public:
	online_logger_sink(OnlineDebugHub& onlineDebugHub, std::string logId)
		: m_Hub(onlineDebugHub)
		  , m_LogId(logId)
	{
	}

protected:
	void sink_it_(const spdlog::details::log_msg& msg) override
	{
		spdlog::memory_buf_t formatted;
		spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
		m_Logs.push_back(fmt::to_string(formatted));
	}

	void flush_() override
	{
		std::thread([this, logs = m_Logs]()
		{
			m_Hub.Invoke("AddLog", m_LogId, logs);
		}).detach();
		m_Logs.clear();
	}
};

using online_logger_sink_mt = online_logger_sink<std::mutex>;
using online_logger_sink_st = online_logger_sink<spdlog::details::null_mutex>;

/**
 * @brief Online debug service
 */
class OnlineDebug
{
	std::unique_ptr<OnlineDebugHub> m_Hub;
	bool m_Valid;
public:
	OSSIAN_SERVICE_SETUP(OnlineDebug())
	{
		m_Valid = false;
	}

	/**
	 * @brief Connect to service
	 * 
	 * @param url URL
	 */
	auto Connect(std::string url)
	{
		m_Hub.reset(new OnlineDebugHub{
			std::move(
				signalr::hub_connection_builder::create(url)
				.with_logging(std::make_shared<SignalRLogger>(), signalr::trace_level::errors)
				.build())
		});
		m_Valid = true;
	}

	/**
	 * @brief Initialize logger.
	 * 
	 * @param loggerName The logger name. In order to access logger through spdlog::get("loggerName").
	 * @param logName The log name to display on online board.
	 * @param logDescription The description to display on online board.
	 * @param argumentId The argument id that connect to the log
	 * @param logLevel Log level
	 * @param isOffline Set true to enable offline log file instead of online debug.
	 * @param offlineLogFilename Filename to store offline log file
	 */
	auto StartLogging(std::string loggerName,
	                  const std::string logName,
	                  const std::string logDescription,
	                  const std::string argumentId,
	                  bool enableStdlog              = true,
	                  int logLevel                   = 0,
	                  bool isOffline                 = false,
	                  std::string offlineLogFilename = "") const -> void
	{
		if (!m_Valid && !isOffline)
		{
			throw std::runtime_error("OnlineDebug is not valid");
		}

		auto distSink = std::make_shared<spdlog::sinks::dist_sink_mt>();
		if (!isOffline)
		{
			std::promise<std::string> waitLogId;
			m_Hub->Invoke("CreateLog", logName, logDescription, argumentId)
			     .Then<std::string>([&waitLogId](const std::string& id, std::exception_ptr)
				     {
					     waitLogId.set_value(std::string{id});
				     });
			auto logId            = waitLogId.get_future().get();
			const auto onlineSink = std::make_shared<online_logger_sink_mt>(*m_Hub, logId);
			distSink->add_sink(onlineSink);
			if (enableStdlog)
			{
				const auto stdSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
				distSink->add_sink(stdSink);
			}
		}
		else
		{
			const auto offlineFileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(offlineLogFilename);
			distSink->add_sink(offlineFileSink);
		}

		auto logger = std::make_shared<spdlog::logger>(loggerName, distSink);
		logger->set_pattern("[%Y-%m-%dT%T.%e%z] [%-5t] %^[%l]%$ %v");
		logger->set_level(static_cast<spdlog::level::level_enum>(logLevel));
		spdlog::register_logger(logger);
		spdlog::set_default_logger(logger);
		std::thread([logger]()
		{
			while (true)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				logger->flush();
			}
		}).detach();
	}

	auto UploadOfflineLog(std::string logFilename,
	                      const std::string logName,
	                      const std::string logDescription,
	                      const std::string argumentId) -> void
	{
		if (!m_Valid)
		{
			throw std::runtime_error("OnlineDebug is not valid");
		}
		const auto stdSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
		auto stdLogger     = std::make_shared<spdlog::logger>("stdLogger", stdSink);
		stdLogger->set_pattern("[%Y-%m-%dT%T.%e%z] [%-5t] %^[%l]%$ %v");

		std::promise<std::string> waitLogId;
		m_Hub->Invoke("CreateLog", logName, logDescription, argumentId)
		     .Then<std::string>([&waitLogId](const std::string& id, std::exception_ptr)
		     {
			     waitLogId.set_value(std::string{id});
		     });
		auto logId = waitLogId.get_future().get();
		stdLogger->info("Create log: {}", logId);


		std::ifstream ifs(logFilename, std::ios::in);
		std::string line{};
		std::vector<std::string> content;
		while (!ifs.eof())
		{
			std::getline(ifs, line);
			if (ifs.fail())
			{
				break;
			}
			content.push_back(line);
			if (content.size() > 100)
			{
				std::promise<bool> waitStatus;
				m_Hub->Invoke("AddLog", logId, content)
				     .Then<bool>([&](bool, std::exception_ptr)
				     {
					     stdLogger->trace("Uploaded: {}", content.size());
					     waitStatus.set_value(true);
				     });
				waitStatus.get_future().get();
				content.clear();
			}
		}
		{
			std::promise<bool> waitStatus;
			m_Hub->Invoke("AddLog", logId, content)
			     .Then<bool>([&](bool, std::exception_ptr)
			     {
				     stdLogger->trace("Uploaded: {}", content.size());
				     waitStatus.set_value(true);
			     });
			waitStatus.get_future().get();
			content.clear();
		}
		stdLogger->info("Upload success");
	}
};

#endif // OSSIAN_ONLINE_DEBUG_HPP
