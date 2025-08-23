#include "DroneController.h"
#include "mission_utils.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

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