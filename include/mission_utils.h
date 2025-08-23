#pragma once

#include "waypoint.h"

// Mission creation utilities
MissionConfig create_square_patrol(double home_lat, double home_lon, float altitude, double side_length_deg = 0.002);
MissionConfig load_mission_from_user();