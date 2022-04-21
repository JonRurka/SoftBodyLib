#pragma once

#include "stdafx.h"
#include "C_Variables.h"



namespace SoftBodyLib
{
	class Logger {
	public:
		enum class Level {
			Error = 3,
			Warning = 2,
			Info = 1,
			Debug = 0
		};

		struct LogEntry {
		public:
			Level log_level;
			std::string source;
			std::string message;
		};


		static void Log(Level level, std::string source, std::string message);

		static void LogError(std::string source, std::string message);
		static void LogWarning(std::string source, std::string message);
		static void LogInfo(std::string source, std::string message);
		static void LogDebug(std::string source, std::string message);

		static std::vector<LogEntry> GetLogEntries();

		

	private:

		static Logger m_logger;
		std::vector<LogEntry> m_entries;
		//std::vector<LogEntry> m_entries_processing;
	};
}

