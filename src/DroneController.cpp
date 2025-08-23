#include "DroneController.h"
#include "logger.h"
#include <cmath>

using namespace mavsdk;

DroneController::DroneController() : mavsdk_(Mavsdk::Configuration(255, 0, true)) {}

bool DroneController::initialize(const std::string& connection_url) {
    Logger::info("Initializing Drone Surveillance System...");
    Logger::debug("Attempting connection to: " + connection_url);

    ConnectionResult connection_result = mavsdk_.add_any_connection(connection_url);
    if (connection_result != ConnectionResult::Success) {
        std::string error_msg = "Connection failed with result: " + std::to_string(static_cast<int>(connection_result));
        Logger::error(error_msg);
        return false;
    }

    Logger::info("Waiting for system to connect...");
    Logger::debug("Looking for PX4 autopilot system (timeout: 30 seconds)");

    for (int i = 0; i < 30; ++i) {
        auto systems = mavsdk_.systems();
        if (!systems.empty()) {
            system_ = systems[0];
            if (system_->has_autopilot()) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (i % 5 == 0) Logger::debug("Connection attempt " + std::to_string(i + 1) + "/30");
    }

    if (!system_ || !system_->has_autopilot()) {
        Logger::error("No autopilot found after 30 second timeout");
        Logger::info("To test with simulation: cd ~/PX4-Autopilot && make px4_sitl_default gazebo-classic");
        return false;
    }

    action_ = std::make_shared<Action>(system_);
    offboard_ = std::make_shared<Offboard>(system_);
    telemetry_ = std::make_shared<Telemetry>(system_);
    mission_ = std::make_shared<Mission>(system_);

    Logger::info("System connected successfully!");
    return true;
}

bool DroneController::arm_and_takeoff(float altitude) {
    Logger::info("Arming vehicle...");

    const Action::Result arm_result = action_->arm();
    if (arm_result != Action::Result::Success) {
        Logger::error("Arming failed: " + std::to_string(static_cast<int>(arm_result)));
        return false;
    }

    Logger::info("Setting takeoff altitude to " + std::to_string(altitude) + "m");
    action_->set_takeoff_altitude(altitude);

    Logger::info("Taking off...");
    const Action::Result takeoff_result = action_->takeoff();
    if (takeoff_result != Action::Result::Success) {
        Logger::error("Takeoff failed: " + std::to_string(static_cast<int>(takeoff_result)));
        return false;
    }

    Logger::info("Waiting for takeoff to complete...");
    std::this_thread::sleep_for(std::chrono::seconds(15));
    Logger::info("Takeoff successful!");
    return true;
}

bool DroneController::goto_waypoint(const Waypoint& waypoint) {
    Logger::info("Flying to waypoint: " + waypoint.description);
    Logger::debug("Coordinates: Lat: " + std::to_string(waypoint.latitude) +
                 ", Lon: " + std::to_string(waypoint.longitude) +
                 ", Alt: " + std::to_string(waypoint.altitude) + "m");

    // Get current position first
    auto start_pos = telemetry_->position();
    Logger::debug("Starting from: Lat: " + std::to_string(start_pos.latitude_deg) +
                 ", Lon: " + std::to_string(start_pos.longitude_deg) +
                 ", Alt: " + std::to_string(start_pos.relative_altitude_m) + "m");

    // Use offboard mode for better altitude control
    using namespace std::chrono_literals;

    // Set the initial setpoint
    Offboard::PositionNedYaw position_ned_yaw{};

    // Convert GPS to NED coordinates (very rough approximation for small distances)
    double lat_diff = waypoint.latitude - start_pos.latitude_deg;
    double lon_diff = waypoint.longitude - start_pos.longitude_deg;

    // Convert degrees to meters (rough approximation)
    position_ned_yaw.north_m = lat_diff * 111000.0;  // ~111km per degree latitude
    position_ned_yaw.east_m = lon_diff * 111000.0 * cos(waypoint.latitude * M_PI / 180.0);  // longitude varies by latitude
    position_ned_yaw.down_m = -(waypoint.altitude); // NED uses negative for altitude
    position_ned_yaw.yaw_deg = NAN; // Don't change heading

    Logger::trace("NED target: N=" + std::to_string(position_ned_yaw.north_m) +
                 "m, E=" + std::to_string(position_ned_yaw.east_m) +
                 "m, D=" + std::to_string(position_ned_yaw.down_m) + "m");

    // Start offboard mode
    offboard_->set_position_ned(position_ned_yaw);

    Offboard::Result offboard_result = offboard_->start();
    if (offboard_result != Offboard::Result::Success) {
        Logger::warn("Offboard start failed, falling back to goto_location");
        const Action::Result goto_result = action_->goto_location(
            waypoint.latitude, waypoint.longitude, waypoint.altitude, NAN);

        if (goto_result != Action::Result::Success) {
            Logger::error("Both offboard and goto_location failed");
            return false;
        }
    }

    Logger::info("Flying to waypoint...");

    auto start_time = std::chrono::steady_clock::now();

    // Wait until we're close to target OR 30 seconds max
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
        auto current_pos = telemetry_->position();

        double lat_diff_current = abs(current_pos.latitude_deg - waypoint.latitude);
        double lon_diff_current = abs(current_pos.longitude_deg - waypoint.longitude);
        double alt_diff_current = abs(current_pos.relative_altitude_m - waypoint.altitude);

        // Check if we're close enough (within 20m horizontally, 2m vertically)
        if (lat_diff_current < 0.0002 && lon_diff_current < 0.0002 && alt_diff_current < 2.0) {
            Logger::info("Reached waypoint! Final position: Lat: " +
                        std::to_string(current_pos.latitude_deg) +
                        ", Lon: " + std::to_string(current_pos.longitude_deg) +
                        ", Alt: " + std::to_string(current_pos.relative_altitude_m) + "m");
            break;
        }

        // Check if drone is losing altitude dangerously
        if (current_pos.relative_altitude_m < 2.0) {
            Logger::critical("Drone altitude too low (" + std::to_string(current_pos.relative_altitude_m) +
                           "m). Stopping offboard mode");
            offboard_->stop();
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Stop offboard mode and return to normal flight
    offboard_->stop();

    // Hold position if specified
    if (waypoint.hold_time_s > 0) {
        Logger::info("Holding position for " + std::to_string(waypoint.hold_time_s) + " seconds...");
        std::this_thread::sleep_for(std::chrono::seconds((int)waypoint.hold_time_s));
    }

    Logger::info("Waypoint completed!");
    return true;
}

bool DroneController::execute_mission(const MissionConfig& mission) {
    Logger::info("Starting mission: " + mission.mission_name);
    Logger::info("Total waypoints: " + std::to_string(mission.waypoints.size()));

    for (size_t i = 0; i < mission.waypoints.size(); ++i) {
        Logger::info("--- Waypoint " + std::to_string(i + 1) + "/" + std::to_string(mission.waypoints.size()) + " ---");

        if (!goto_waypoint(mission.waypoints[i])) {
            Logger::error("Failed to reach waypoint " + std::to_string(i + 1));
            return false;
        }
    }

    Logger::info("Mission completed successfully!");
    return true;
}

bool DroneController::return_to_home() {
    Logger::info("Returning to home position...");
    const Action::Result rth_result = action_->return_to_launch();
    if (rth_result != Action::Result::Success) {
        Logger::error("Return to home failed: " + std::to_string(static_cast<int>(rth_result)));
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(15));
    return true;
}

bool DroneController::land_and_disarm() {
    Logger::info("Landing...");
    const Action::Result land_result = action_->land();
    if (land_result != Action::Result::Success) {
        Logger::error("Landing failed: " + std::to_string(static_cast<int>(land_result)));
        return false;
    }

    while (telemetry_->armed()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    Logger::info("Landing and disarming successful!");
    return true;
}

Telemetry::Position DroneController::get_current_position() {
    return telemetry_->position();
}

void DroneController::start_telemetry() {
    telemetry_->subscribe_position([](Telemetry::Position position) {
        Logger::trace("[TELEMETRY] Position: Lat: " + std::to_string(position.latitude_deg) +
                     ", Lon: " + std::to_string(position.longitude_deg) +
                     ", Alt: " + std::to_string(position.relative_altitude_m) + "m");
    });

    telemetry_->subscribe_battery([](Telemetry::Battery battery) {
        Logger::trace("[TELEMETRY] Battery: " + std::to_string(battery.remaining_percent * 100.0f) + "%");
    });
}

void DroneController::stop_telemetry() {
    // Unsubscribe from telemetry to stop console spam
    telemetry_->subscribe_position(nullptr);
    telemetry_->subscribe_battery(nullptr);
}

void DroneController::print_current_status() {
    auto pos = telemetry_->position();
    auto battery = telemetry_->battery();
    Logger::info("[STATUS] Position: Lat: " + std::to_string(pos.latitude_deg) +
                ", Lon: " + std::to_string(pos.longitude_deg) +
                ", Alt: " + std::to_string(pos.relative_altitude_m) + "m" +
                " | Battery: " + std::to_string(battery.remaining_percent * 100.0f) + "%");
}

bool DroneController::is_ready_for_flight() {
    auto health = telemetry_->health();
    Logger::info("Health Check:");
    Logger::info("  GPS: " + std::string(health.is_global_position_ok ? "OK" : "FAIL"));
    Logger::info("  Home Position: " + std::string(health.is_home_position_ok ? "OK" : "FAIL"));

    return health.is_global_position_ok && health.is_home_position_ok;
}