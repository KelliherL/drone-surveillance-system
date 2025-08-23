#include "DroneController.h"
#include <cmath>

using namespace mavsdk;

DroneController::DroneController() : mavsdk_(Mavsdk::Configuration(255, 0, true)) {}

bool DroneController::initialize(const std::string& connection_url) {
    std::cout << "Initializing Drone Surveillance System..." << std::endl;

    ConnectionResult connection_result = mavsdk_.add_any_connection(connection_url);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "ERROR: Connection failed: " << connection_result << std::endl;
        return false;
    }

    std::cout << "Waiting for system to connect..." << std::endl;

    for (int i = 0; i < 30; ++i) {
        auto systems = mavsdk_.systems();
        if (!systems.empty()) {
            system_ = systems[0];
            if (system_->has_autopilot()) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "." << std::flush;
    }

    if (!system_ || !system_->has_autopilot()) {
        std::cerr << "\nERROR: No autopilot found, timing out." << std::endl;
        return false;
    }

    action_ = std::make_shared<Action>(system_);
    offboard_ = std::make_shared<Offboard>(system_);
    telemetry_ = std::make_shared<Telemetry>(system_);
    mission_ = std::make_shared<Mission>(system_);

    std::cout << "\nSystem connected successfully!" << std::endl;
    return true;
}

bool DroneController::arm_and_takeoff(float altitude) {
    std::cout << "Arming vehicle..." << std::endl;

    const Action::Result arm_result = action_->arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "ERROR: Arming failed: " << arm_result << std::endl;
        return false;
    }

    std::cout << "Setting takeoff altitude to " << altitude << "m..." << std::endl;
    action_->set_takeoff_altitude(altitude);

    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action_->takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "ERROR: Takeoff failed: " << takeoff_result << std::endl;
        return false;
    }

    std::cout << "Waiting for takeoff to complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));
    std::cout << "Takeoff successful!" << std::endl;
    return true;
}

bool DroneController::goto_waypoint(const Waypoint& waypoint) {
    std::cout << "Flying to waypoint: " << waypoint.description << std::endl;
    std::cout << "  Coordinates: "
              << std::fixed << std::setprecision(6)
              << "Lat: " << waypoint.latitude
              << ", Lon: " << waypoint.longitude
              << ", Alt: " << waypoint.altitude << "m" << std::endl;

    // Get current position first
    auto start_pos = telemetry_->position();
    std::cout << "  Starting from: Lat: " << start_pos.latitude_deg
              << ", Lon: " << start_pos.longitude_deg
              << ", Alt: " << start_pos.relative_altitude_m << "m" << std::endl;

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

    std::cout << "  NED target: N=" << position_ned_yaw.north_m
              << "m, E=" << position_ned_yaw.east_m
              << "m, D=" << position_ned_yaw.down_m << "m" << std::endl;

    // Start offboard mode
    offboard_->set_position_ned(position_ned_yaw);

    Offboard::Result offboard_result = offboard_->start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "ERROR: Offboard start failed: " << offboard_result << std::endl;
        // Fall back to goto_location
        std::cout << "  Falling back to goto_location method..." << std::endl;
        const Action::Result goto_result = action_->goto_location(
            waypoint.latitude, waypoint.longitude, waypoint.altitude, NAN);

        if (goto_result != Action::Result::Success) {
            std::cerr << "ERROR: Goto location also failed: " << goto_result << std::endl;
            return false;
        }
    }

    // Wait for arrival - check actual position
    std::cout << "  Flying to waypoint..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    // Wait until we're close to target OR 30 seconds max
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
        auto current_pos = telemetry_->position();

        double lat_diff_current = abs(current_pos.latitude_deg - waypoint.latitude);
        double lon_diff_current = abs(current_pos.longitude_deg - waypoint.longitude);
        double alt_diff_current = abs(current_pos.relative_altitude_m - waypoint.altitude);

        // Check if we're close enough (within 20m horizontally, 2m vertically)
        if (lat_diff_current < 0.0002 && lon_diff_current < 0.0002 && alt_diff_current < 2.0) {
            std::cout << "  Reached waypoint! Final position: Lat: " << current_pos.latitude_deg
                      << ", Lon: " << current_pos.longitude_deg
                      << ", Alt: " << current_pos.relative_altitude_m << "m" << std::endl;
            break;
        }

        // Check if drone is losing altitude dangerously
        if (current_pos.relative_altitude_m < 2.0) {
            std::cout << "  WARNING: Drone altitude too low (" << current_pos.relative_altitude_m
                      << "m). Stopping offboard mode." << std::endl;
            offboard_->stop();
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Stop offboard mode and return to normal flight
    offboard_->stop();

    // Hold position if specified
    if (waypoint.hold_time_s > 0) {
        std::cout << "  Holding position for " << waypoint.hold_time_s << " seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds((int)waypoint.hold_time_s));
    }

    std::cout << "  Waypoint completed!" << std::endl;
    return true;
}

bool DroneController::execute_mission(const MissionConfig& mission) {
    std::cout << "Starting mission: " << mission.mission_name << std::endl;
    std::cout << "Total waypoints: " << mission.waypoints.size() << std::endl;

    for (size_t i = 0; i < mission.waypoints.size(); ++i) {
        std::cout << "\n--- Waypoint " << (i + 1) << "/" << mission.waypoints.size() << " ---" << std::endl;

        if (!goto_waypoint(mission.waypoints[i])) {
            std::cerr << "Failed to reach waypoint " << (i + 1) << std::endl;
            return false;
        }
    }

    std::cout << "\nMission completed successfully!" << std::endl;
    return true;
}

bool DroneController::return_to_home() {
    std::cout << "Returning to home position..." << std::endl;
    const Action::Result rth_result = action_->return_to_launch();
    if (rth_result != Action::Result::Success) {
        std::cerr << "ERROR: Return to home failed: " << rth_result << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(15));
    return true;
}

bool DroneController::land_and_disarm() {
    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action_->land();
    if (land_result != Action::Result::Success) {
        std::cerr << "ERROR: Landing failed: " << land_result << std::endl;
        return false;
    }

    while (telemetry_->armed()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Landing and disarming successful!" << std::endl;
    return true;
}

Telemetry::Position DroneController::get_current_position() {
    return telemetry_->position();
}

void DroneController::start_telemetry() {
    telemetry_->subscribe_position([](Telemetry::Position position) {
        std::cout << "[TELEMETRY] Position: "
                 << "Lat: " << std::fixed << std::setprecision(6) << position.latitude_deg
                 << ", Lon: " << std::fixed << std::setprecision(6) << position.longitude_deg
                 << ", Alt: " << std::fixed << std::setprecision(1) << position.relative_altitude_m << "m" << std::endl;
    });

    telemetry_->subscribe_battery([](Telemetry::Battery battery) {
        std::cout << "[TELEMETRY] Battery: " << std::fixed << std::setprecision(1)
                  << battery.remaining_percent * 100.0f << "%" << std::endl;
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
    std::cout << "[STATUS] Position: "
              << "Lat: " << std::fixed << std::setprecision(6) << pos.latitude_deg
              << ", Lon: " << std::fixed << std::setprecision(6) << pos.longitude_deg
              << ", Alt: " << std::fixed << std::setprecision(1) << pos.relative_altitude_m << "m"
              << " | Battery: " << std::fixed << std::setprecision(1) << battery.remaining_percent * 100.0f << "%" << std::endl;
}

bool DroneController::is_ready_for_flight() {
    auto health = telemetry_->health();
    std::cout << "Health Check:" << std::endl;
    std::cout << "  GPS: " << (health.is_global_position_ok ? "OK" : "FAIL") << std::endl;
    std::cout << "  Home Position: " << (health.is_home_position_ok ? "OK" : "FAIL") << std::endl;

    return health.is_global_position_ok && health.is_home_position_ok;
}