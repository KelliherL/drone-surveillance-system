#pragma once

#include <string>

// Enhanced error handling with context
enum class DroneResult {
    SUCCESS = 0,
    CONNECTION_FAILED,
    AUTOPILOT_NOT_FOUND,
    ARM_FAILED,
    TAKEOFF_FAILED,
    WAYPOINT_FAILED,
    LANDING_FAILED,
    OFFBOARD_FAILED,
    TIMEOUT,
    GPS_NOT_READY,
    MISSION_CANCELLED,
    ALTITUDE_TOO_LOW,
    UNKNOWN_ERROR
};

// Convert DroneResult to string
std::string result_to_string(DroneResult result);

// Simple result wrapper
struct OperationResult {
    DroneResult result;
    std::string message;

    // Constructors
    OperationResult(DroneResult r) : result(r), message("") {}
    OperationResult(DroneResult r, const std::string& msg) : result(r), message(msg) {}

    // Status checks
    bool success() const {
        return result == DroneResult::SUCCESS;
    }

    bool failed() const {
        return !success();
    }

    // Get error message
    std::string get_message() const {
        if (!message.empty()) {
            return message;
        }
        return result_to_string(result);
    }

    // Static factory methods
    static OperationResult make_success() {
        return OperationResult(DroneResult::SUCCESS);
    }

    static OperationResult make_error(DroneResult r, const std::string& msg = "") {
        return OperationResult(r, msg);
    }
};