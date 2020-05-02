/**
 * @file OnlineDebug.hpp
 * @author Xu Zihan (mrxzh@outlook.com)
 * @brief Online debug logic
 * @version 0.1
 * @date 2020-02-23
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_ONLINE_DEBUG_HPP
#define OSSIAN_ONLINE_DEBUG_HPP

#include <exception>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <ossian/Factory.hpp>
#include <ossian/Configuration.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <grpcpp/grpcpp.h>

#include "Log.grpc.pb.h"
#include "Config.grpc.pb.h"

class OnlineDebugException : public std::runtime_error
{
public:
	OnlineDebugException(std::string what) : std::runtime_error(what)
	{
	}
};


/**
 * @brief RPC hub (for server calling client).
 */
class OnlineDebugRPC
{
	std::unique_ptr<OssianConfig::OnlineLogService::Stub> m_Stub;
	std::unique_ptr<OssianConfig::ConfigurationService::Stub> m_ConfigServiceStub;

public:
	OnlineDebugRPC(std::shared_ptr<grpc::Channel> channel)
		: m_Stub(OssianConfig::OnlineLogService::NewStub(channel))
		  , m_ConfigServiceStub(OssianConfig::ConfigurationService::NewStub(channel))
	{
	}

	auto CreateLog(std::string name,
	               std::string description) -> std::string
	{
		grpc::ClientContext context;
		OssianConfig::CreateLogRequest request;
		OssianConfig::CreateLogResponse response;
		request.set_name(name);
		request.set_description(description);
		auto status = m_Stub->CreateLog(&context, request, &response);
		spdlog::error("Status: {}", status.error_message());
		return response.logid();
	}

	auto ArchiveConfiguration(std::string logId,
	                          const OssianConfig::Configuration& config) -> void
	{
		grpc::ClientContext context;
		OssianConfig::ArchiveConfigurationRequest request;
		OssianConfig::ArchiveConfigurationResponse response;
		request.set_logid(logId);
		request.mutable_config()->CopyFrom(config);
		m_ConfigServiceStub->ArchiveConfiguration(&context, request, &response);
	}

	auto AddLog(std::string logId,
	            const OssianConfig::AddLogRequest& logs)
	{
		grpc::ClientContext context;
		OssianConfig::AddLogResponse response;
		auto writer = m_Stub->AddLog(&context, logs, &response);
	}

	auto AddLog(std::string logId,
	            const std::vector<std::string>& logs)
	{
		OssianConfig::AddLogRequest request;
		request.mutable_log()->Reserve(logs.size());
		for (auto&& item : logs)
		{
			request.add_log(item);
		}
		AddLog(logId, request);
	}

	auto AddLogAsync(std::string logId,
	                 const OssianConfig::AddLogRequest& logs)
	{
		std::thread([this, logId, logs]()
		{
			AddLog(logId, logs);
		}).detach();
	}

	auto AddLogAsync(std::string logId,
	                 const std::vector<std::string>& logs)
	{
		std::thread([this, logId, logs]()
		{
			AddLog(logId, logs);
		}).detach();
	}

	auto GetConfiguration()
	{
		
	}
};

/**
 * @brief Log sink.
 * 
 * @tparam Mutex Mutex type.
 */
template <typename Mutex>
class online_logger_sink : public spdlog::sinks::base_sink<Mutex>
{
	OnlineDebugRPC& m_Hub;
	OssianConfig::AddLogRequest m_Logs;
	std::string m_LogId;
public:
	online_logger_sink(OnlineDebugRPC& onlineDebugHub, std::string logId)
		: m_Hub(onlineDebugHub)
		  , m_LogId(logId)
	{
	}

protected:
	void sink_it_(const spdlog::details::log_msg& msg) override
	{
		spdlog::memory_buf_t formatted;
		spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
		m_Logs.add_log(fmt::to_string(formatted));
	}

	void flush_() override
	{
		m_Logs.set_logid(m_LogId);
		m_Hub.AddLogAsync(m_LogId, m_Logs);
		m_Logs.clear_log();
	}
};

using online_logger_sink_mt = online_logger_sink<std::mutex>;
using online_logger_sink_st = online_logger_sink<spdlog::details::null_mutex>;

/**
 * @brief Online debug service
 */
class OnlineDebug
{
	std::unique_ptr<OnlineDebugRPC> m_Hub;
	std::shared_ptr<grpc::Channel> m_Channel;
	bool m_Valid;
public:
	OSSIAN_SERVICE_SETUP(OnlineDebug())
	{
		m_Valid = false;
	}

	void ReadCert(const std::string& filename, std::string& data)
	{
		std::ifstream file(filename.c_str(), std::ios::in);
		if (file.is_open())
		{
			std::stringstream ss;
			ss << file.rdbuf();

			file.close();

			data = ss.str();
		}

		return;
	}
	
	/**
	 * @brief Connect to service
	 *
	 * @param url URL
	 */
	auto Connect(std::string url)
	{
		std::string cert;
		ReadCert("key.crt", cert);
		auto channelCreds = grpc::SslCredentials(grpc::SslCredentialsOptions{cert});
		m_Channel         = grpc::CreateChannel(url, channelCreds);
		m_Hub.reset(new OnlineDebugRPC(m_Channel));
		m_Valid = true;
	}

	/**
	 * @brief Initialize logger.
	 *
	 * @param loggerName The logger name. In order to access logger through spdlog::get("loggerName").
	 * @param logName The log name to display on online board.
	 * @param logDescription The description to display on online board.
	 */
	auto StartLogging(std::string loggerName,
	                  std::string logName,
	                  std::string logDescription) -> std::string
	{
		if (!m_Valid)
		{
			throw std::runtime_error("OnlineDebug is not valid");
		}
		const auto logId = m_Hub->CreateLog(logName, logDescription);
		
		auto distSink   = std::make_shared<spdlog::sinks::dist_sink_mt>();
		auto stdSink    = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
		auto onlineSink = std::make_shared<online_logger_sink_mt>(*m_Hub, logId);

		distSink->add_sink(stdSink);
		distSink->add_sink(onlineSink);

		auto logger = std::make_shared<spdlog::logger>(loggerName, distSink);
		logger->set_pattern("[%Y-%m-%dT%T.%e%z] [%-5t] %^[%l]%$ %v");
		logger->set_level(spdlog::level::level_enum::trace);
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

		return logId;
	}

	auto StartLoggingAndArchiveLog(std::string loggerName,
								   std::string logName,
								   std::string logDescription,
								   OssianConfig::Configuration config)
	{
		const auto logId = StartLogging(loggerName, logName, logDescription);
		m_Hub->ArchiveConfiguration(logId, config);
	}
};

#endif // OSSIAN_ONLINE_DEBUG_HPP
