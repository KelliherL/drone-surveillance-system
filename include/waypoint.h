#pragma once

#include <string>
#include <vector>
#include <map>

// Simple waypoint structure for dev purposes
struct Waypoint {
    double latitude;
    double longitude;
    float altitude;
    float hold_time_s = 0.0f;
    std::string description;

    // Constructor for easy creation
    Waypoint(double lat, double lon, float alt, const std::string& desc = "")
        : latitude(lat), longitude(lon), altitude(alt), description(desc) {}

    // Default constructor
    Waypoint() : latitude(0), longitude(0), altitude(0) {}
};

// Simple mission configuration
struct MissionConfig {
    std::vector<Waypoint> waypoints;
    float default_altitude = 15.0f;
    std::string mission_name = "Unnamed Mission";

    // Simple file operations (Note: implement these later)
    bool save_to_file(const std::string& filename) const;
    bool load_from_file(const std::string& filename);
};