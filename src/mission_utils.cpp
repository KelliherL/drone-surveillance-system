#include "mission_utils.h"
#include <iostream>
#include <iomanip>

MissionConfig create_square_patrol(double home_lat, double home_lon, float altitude, double side_length_deg) {
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