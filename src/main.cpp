#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;

class DroneController {
private:
    Mavsdk mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Action> action_;
    std::shared_ptr<Offboard> offboard_;
    std::shared_ptr<Telemetry> telemetry_;

public:
    DroneController() : mavsdk_(Mavsdk::Configuration(255, 0, true)) {}

    ~DroneController() = default;

    bool initialize(const std::string& connection_url = "udp://:14540") {
        std::cout << "Initializing Drone Surveillance System..." << std::endl;

        // Add connection
        ConnectionResult connection_result = mavsdk_.add_any_connection(connection_url);
        if (connection_result != ConnectionResult::Success) {
            std::cerr << "ERROR: Connection failed: " << connection_result << std::endl;
            return false;
        }

        std::cout << "Waiting for system to connect..." << std::endl;

        // Wait for system discovery
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

        // Initialize plugins
        action_ = std::make_shared<Action>(system_);
        offboard_ = std::make_shared<Offboard>(system_);
        telemetry_ = std::make_shared<Telemetry>(system_);

        std::cout << "\nSystem connected successfully!" << std::endl;
        return true;
    }

    bool arm_and_takeoff() {
        std::cout << "Arming vehicle..." << std::endl;

        const Action::Result arm_result = action_->arm();
        if (arm_result != Action::Result::Success) {
            std::cerr << "ERROR: Arming failed: " << arm_result << std::endl;
            return false;
        }

        std::cout << "Taking off..." << std::endl;

        const Action::Result takeoff_result = action_->takeoff();
        if (takeoff_result != Action::Result::Success) {
            std::cerr << "ERROR: Takeoff failed: " << takeoff_result << std::endl;
            return false;
        }

        // Wait for takeoff to complete
        std::this_thread::sleep_for(std::chrono::seconds(10));
        std::cout << "Takeoff successful!" << std::endl;
        return true;
    }

    bool land_and_disarm() {
        std::cout << "Landing..." << std::endl;

        const Action::Result land_result = action_->land();
        if (land_result != Action::Result::Success) {
            std::cerr << "ERROR: Landing failed: " << land_result << std::endl;
            return false;
        }

        // Wait for landing
        while (telemetry_->armed()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "Landing and disarming successful!" << std::endl;
        return true;
    }

    void print_telemetry() {
        // Subscribe to telemetry
        telemetry_->subscribe_position([](Telemetry::Position position) {
            std::cout << "Position: "
                     << "Lat: " << position.latitude_deg
                     << ", Lon: " << position.longitude_deg
                     << ", Alt: " << position.relative_altitude_m << "m" << std::endl;
        });

        telemetry_->subscribe_battery([](Telemetry::Battery battery) {
            std::cout << "Battery: " << battery.remaining_percent * 100.0f << "%" << std::endl;
        });
    }
};

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "        DRONE SURVEILLANCE SYSTEM           " << std::endl;
    std::cout << "===============================================" << std::endl;

    DroneController drone;

    // Initialize connection
    if (!drone.initialize()) {
        std::cerr << "Failed to initialize drone controller" << std::endl;
        return 1;
    }

    // Start telemetry monitoring
    drone.print_telemetry();

    // Basic flight test
    if (!drone.arm_and_takeoff()) {
        std::cerr << "Failed to arm and takeoff" << std::endl;
        return 1;
    }

    // Hold position for surveillance
    std::cout << "Starting surveillance mode..." << std::endl;
    std::cout << "Holding position for 30 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(30));

    // Return to base
    if (!drone.land_and_disarm()) {
        std::cerr << "Failed to land and disarm" << std::endl;
        return 1;
    }

    std::cout << "Mission completed successfully!" << std::endl;
    return 0;
}