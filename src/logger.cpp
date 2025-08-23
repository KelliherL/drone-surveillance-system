#include "logger.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>

// Use basic logging for now - no spdlog dependency
LogLevel Logger::current_level_ = LogLevel::INFO;
std::ofstream Logger::log_file_;
bool Logger::initialized_ = false;

void Logger::initialize(const std::string& log_file_path, LogLevel level) {
    if (initialized_) return;

    current_level_ = level;
    log_file_.open(log_file_path, std::ios::app);
    initialized_ = true;
    info("Logging system initialized");
}

void Logger::set_level(LogLevel level) {
    current_level_ = level;
}

void Logger::trace(const std::string& message) {
    if (!initialized_) initialize();
    log_basic(LogLevel::TRACE, message);
}

void Logger::debug(const std::string& message) {
    if (!initialized_) initialize();
    log_basic(LogLevel::DEBUG, message);
}

void Logger::info(const std::string& message) {
    if (!initialized_) initialize();
    log_basic(LogLevel::INFO, message);
}

void Logger::warn(const std::string& message) {
    if (!initialized_) initialize();
    log_basic(LogLevel::WARN, message);
}

void Logger::error(const std::string& message) {
    if (!initialized_) initialize();
    log_basic(LogLevel::ERROR, message);
}

void Logger::critical(const std::string& message) {
    if (!initialized_) initialize();
    log_basic(LogLevel::CRITICAL, message);
}

void Logger::log_basic(LogLevel level, const std::string& message) {
    if (level < current_level_) return;

    std::string timestamp = get_timestamp();
    std::string level_str = level_to_string(level);
    std::string log_message = "[" + timestamp + "] [" + level_str + "] " + message;

    // Output to console
    std::cout << log_message << std::endl;

    // Output to file if available
    if (log_file_.is_open()) {
        log_file_ << log_message << std::endl;
        log_file_.flush();
    }
}

std::string Logger::level_to_string(LogLevel level) {
    switch (level) {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARN: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

std::string Logger::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    return ss.str();
}