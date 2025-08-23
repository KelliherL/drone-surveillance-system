#include "result_types.h"

std::string result_to_string(DroneResult result) {
    switch (result) {
        case DroneResult::SUCCESS: return "Success";
        case DroneResult::CONNECTION_FAILED: return "Connection failed";
        case DroneResult::AUTOPILOT_NOT_FOUND: return "Autopilot not found";
        case DroneResult::ARM_FAILED: return "Failed to arm";
        case DroneResult::TAKEOFF_FAILED: return "Takeoff failed";
        case DroneResult::WAYPOINT_FAILED: return "Waypoint navigation failed";
        case DroneResult::LANDING_FAILED: return "Landing failed";
        case DroneResult::OFFBOARD_FAILED: return "Offboard mode failed";
        case DroneResult::TIMEOUT: return "Operation timed out";
        case DroneResult::GPS_NOT_READY: return "GPS not ready";
        case DroneResult::MISSION_CANCELLED: return "Mission cancelled by user";
        case DroneResult::ALTITUDE_TOO_LOW: return "Altitude too low for safety";
        case DroneResult::UNKNOWN_ERROR: return "Unknown error occurred";
        default: return "Undefined error";
    }
}