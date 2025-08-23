#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <fstream>

enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    CRITICAL = 5
};

class Logger {
private:
    static LogLevel current_level_;
    static std::ofstream log_file_;
    static bool initialized_;

public:
    static void initialize(const std::string& log_file_path = "drone_system.log",
                          LogLevel level = LogLevel::INFO);

    static void set_level(LogLevel level);

    // Simple logging methods
    static void trace(const std::string& message);
    static void debug(const std::string& message);
    static void info(const std::string& message);
    static void warn(const std::string& message);
    static void error(const std::string& message);
    static void critical(const std::string& message);

private:
    static void log_basic(LogLevel level, const std::string& message);
    static std::string level_to_string(LogLevel level);
    static std::string get_timestamp();
};