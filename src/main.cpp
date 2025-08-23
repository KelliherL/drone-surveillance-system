#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>

// Include our waypoint structure
#include "waypoint.h"

using namespace mavsdk;

class DroneController {
private:
    Mavsdk mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Action> action_;
    std::shared_ptr<Offboard> offboard_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Mission> mission_;

public:
    DroneController() : mavsdk_(Mavsdk::Configuration(255, 0, true)) {}

    bool initialize(const std::string& connection_url = "udp://:14540") {
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

    bool arm_and_takeoff(float altitude = 15.0f) {
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

    bool goto_waypoint(const Waypoint& waypoint) {
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

    bool execute_mission(const MissionConfig& mission) {
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

    bool return_to_home() {
        std::cout << "Returning to home position..." << std::endl;
        const Action::Result rth_result = action_->return_to_launch();
        if (rth_result != Action::Result::Success) {
            std::cerr << "ERROR: Return to home failed: " << rth_result << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(15));
        return true;
    }

    bool land_and_disarm() {
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

    Telemetry::Position get_current_position() {
        return telemetry_->position();
    }

    void start_telemetry() {
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

    void stop_telemetry() {
        // Unsubscribe from telemetry to stop console spam
        telemetry_->subscribe_position(nullptr);
        telemetry_->subscribe_battery(nullptr);
    }

    void print_current_status() {
        auto pos = telemetry_->position();
        auto battery = telemetry_->battery();
        std::cout << "[STATUS] Position: "
                  << "Lat: " << std::fixed << std::setprecision(6) << pos.latitude_deg
                  << ", Lon: " << std::fixed << std::setprecision(6) << pos.longitude_deg
                  << ", Alt: " << std::fixed << std::setprecision(1) << pos.relative_altitude_m << "m"
                  << " | Battery: " << std::fixed << std::setprecision(1) << battery.remaining_percent * 100.0f << "%" << std::endl;
    }

    bool is_ready_for_flight() {
        auto health = telemetry_->health();
        std::cout << "Health Check:" << std::endl;
        std::cout << "  GPS: " << (health.is_global_position_ok ? "OK" : "FAIL") << std::endl;
        std::cout << "  Home Position: " << (health.is_home_position_ok ? "OK" : "FAIL") << std::endl;

        return health.is_global_position_ok && health.is_home_position_ok;
    }
};

// Helper functions for creating missions
MissionConfig create_square_patrol(double home_lat, double home_lon, float altitude, double side_length_deg = 0.002) {
    MissionConfig mission;
    mission.mission_name = "Square Patrol";
    mission.default_altitude = altitude;

    std::cout << "Creating square patrol around home position:" << std::endl;
    std::cout << "  Home: " << std::fixed << std::setprecision(6) << home_lat << ", " << home_lon << std::endl;
    std::cout << "  Square size: " << side_length_deg << " degrees (~" << (side_length_deg * 111000) << " meters)" << std::endl;

    mission.waypoints = {
        Waypoint(home_lat + side_length_deg, home_lon, altitude, "North Point"),
        Waypoint(home_lat + side_length_deg, home_lon + side_length_deg, altitude, "Northeast Point"),
        Waypoint(home_lat, home_lon + side_length_deg, altitude, "East Point"),
        Waypoint(home_lat, home_lon, altitude, "Return Home")
    };

    // Add hold time for surveillance
    for (auto& wp : mission.waypoints) {
        wp.hold_time_s = 5.0f; // Reduced from 10 to 5 seconds
    }

    return mission;
}

MissionConfig load_mission_from_user() {
    MissionConfig mission;
    mission.mission_name = "Custom Mission";

    std::cout << "\n=== MISSION BUILDER ===" << std::endl;
    std::cout << "Enter mission name: ";
    std::getline(std::cin, mission.mission_name);

    std::cout << "Enter default altitude (m): ";
    std::cin >> mission.default_altitude;

    int num_waypoints;
    std::cout << "Enter number of waypoints: ";
    std::cin >> num_waypoints;

    for (int i = 0; i < num_waypoints; ++i) {
        Waypoint wp;
        std::cout << "\n--- Waypoint " << (i + 1) << " ---" << std::endl;
        std::cout << "Latitude: ";
        std::cin >> wp.latitude;
        std::cout << "Longitude: ";
        std::cin >> wp.longitude;
        std::cout << "Altitude (m): ";
        std::cin >> wp.altitude;
        std::cout << "Hold time (s): ";
        std::cin >> wp.hold_time_s;

        std::cin.ignore(); // Clear newline
        std::cout << "Description: ";
        std::getline(std::cin, wp.description);

        mission.waypoints.push_back(wp);
    }

    return mission;
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "        DRONE SURVEILLANCE SYSTEM           " << std::endl;
    std::cout << "===============================================" << std::endl;

    DroneController drone;

    if (!drone.initialize()) {
        std::cerr << "Failed to initialize drone controller" << std::endl;
        return 1;
    }

    // Start telemetry but give it a moment to stabilize
    drone.start_telemetry();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Stop the continuous telemetry spam for user interaction
    drone.stop_telemetry();

    // Print current status once
    drone.print_current_status();

    if (!drone.is_ready_for_flight()) {
        std::cerr << "Drone is not ready for flight. Check GPS and systems." << std::endl;
        return 1;
    }

    // Mission selection
    std::cout << "\nMission Options:" << std::endl;
    std::cout << "1. Automated square patrol" << std::endl;
    std::cout << "2. Custom waypoint mission" << std::endl;
    std::cout << "Choose option (1 or 2): ";

    int choice;
    std::cin >> choice;
    std::cin.ignore(); // Clear newline

    MissionConfig mission;

    if (choice == 1) {
        // Automated mission
        auto home_position = drone.get_current_position();
        mission = create_square_patrol(home_position.latitude_deg, home_position.longitude_deg, 15.0f);
    } else if (choice == 2) {
        // User-defined mission
        mission = load_mission_from_user();
    } else {
        std::cerr << "Invalid choice" << std::endl;
        return 1;
    }

    std::cout << "\n=== MISSION SUMMARY ===" << std::endl;
    std::cout << "Mission: " << mission.mission_name << std::endl;
    std::cout << "Waypoints: " << mission.waypoints.size() << std::endl;
    for (size_t i = 0; i < mission.waypoints.size(); ++i) {
        const auto& wp = mission.waypoints[i];
        std::cout << "  " << (i + 1) << ". " << wp.description
                  << " (" << std::fixed << std::setprecision(6) << wp.latitude
                  << ", " << std::fixed << std::setprecision(6) << wp.longitude
                  << ", " << wp.altitude << "m)" << std::endl;
    }

    std::cout << "\nProceed with mission? (y/n): ";
    char confirm;
    std::cin >> confirm;

    if (confirm != 'y' && confirm != 'Y') {
        std::cout << "Mission cancelled." << std::endl;
        return 0;
    }

    // Execute mission
    std::cout << "\nStarting mission execution..." << std::endl;

    // Restart telemetry for flight monitoring
    drone.start_telemetry();

    if (!drone.arm_and_takeoff(mission.default_altitude)) {
        std::cerr << "Failed to arm and takeoff" << std::endl;
        return 1;
    }

    if (!drone.execute_mission(mission)) {
        std::cerr << "Mission execution failed" << std::endl;
        drone.return_to_home();
        return 1;
    }

    if (!drone.return_to_home()) {
        std::cerr << "Failed to return to home" << std::endl;
    }

    if (!drone.land_and_disarm()) {
        std::cerr << "Failed to land and disarm" << std::endl;
        return 1;
    }

    std::cout << "Mission completed successfully!" << std::endl;
    return 0;
}