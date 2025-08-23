#pragma once

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

#include "waypoint.h"
#include "logger.h"

class DroneController {
private:
    mavsdk::Mavsdk mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::shared_ptr<mavsdk::Action> action_;
    std::shared_ptr<mavsdk::Offboard> offboard_;
    std::shared_ptr<mavsdk::Telemetry> telemetry_;
    std::shared_ptr<mavsdk::Mission> mission_;

public:
    DroneController();

    // Core initialization and connection
    bool initialize(const std::string& connection_url = "udp://:14540");

    // Flight control methods
    bool arm_and_takeoff(float altitude = 15.0f);
    bool goto_waypoint(const Waypoint& waypoint);
    bool execute_mission(const MissionConfig& mission);
    bool return_to_home();
    bool land_and_disarm();

    // Telemetry and status
    mavsdk::Telemetry::Position get_current_position();
    void start_telemetry();
    void stop_telemetry();
    void print_current_status();
    bool is_ready_for_flight();
};