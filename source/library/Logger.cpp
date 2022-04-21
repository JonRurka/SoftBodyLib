#include "Logger.h"

using namespace SoftBodyLib;

SoftBodyLib::Logger Logger::m_logger;

void SoftBodyLib::Logger::Log(Logger::Level level, std::string source, std::string message)
{
	LogEntry entry;
	entry.log_level = level;
	entry.source = source;
	entry.message = message;

	m_logger.m_entries.push_back(entry);
}

void SoftBodyLib::Logger::LogError(std::string source, std::string message)
{
	Log(Level::Error, source, message);
}

void SoftBodyLib::Logger::LogWarning(std::string source, std::string message)
{
	Log(Level::Warning, source, message);
}

void SoftBodyLib::Logger::LogInfo(std::string source, std::string message)
{
	Log(Level::Info, source, message);
}

void SoftBodyLib::Logger::LogDebug(std::string source, std::string message)
{
	Log(Level::Debug, source, message);
}

std::vector<Logger::LogEntry> SoftBodyLib::Logger::GetLogEntries()
{
	// Declaring new vector
	std::vector<Logger::LogEntry> res;

	// Copying vector by copy function
	std::copy(m_logger.m_entries.begin(), m_logger.m_entries.end(), std::back_inserter(res));

	m_logger.m_entries.clear();

	return res;
}


