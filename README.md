# Drone Surveillance System

A comprehensive drone surveillance solution built with MAVSDK for autonomous flight control, real-time computer vision, and intelligent monitoring capabilities.

## Project Overview

This system provides:
- Autonomous drone flight control using MAVSDK
- Real-time video streaming and recording
- Computer vision for object detection and tracking
- Ground control station with live monitoring
- Automated surveillance patterns and alerts

## Technology Stack

- **Flight Control**: MAVSDK (C++)
- **Computer Vision**: OpenCV, Python
- **Simulation**: Gazebo
- **Build System**: CMake
- **Platform**: Ubuntu 20.04/22.04

## Prerequisites

- Ubuntu 20.04+ 
- CMake 3.16+
- GCC 7.0+
- Python 3.8+
- MAVSDK installed system-wide
- Gazebo simulator
- PX4 Autopilot (for simulation)

## Quick Start

### Dependencies Installation

```bash
# Install build tools
sudo apt update
sudo apt install -y build-essential cmake git libcurl4-openssl-dev libssl-dev

# Install MAVSDK (choose one method):

# Method 1: From releases (recommended)
wget https://github.com/mavlink/MAVSDK/releases/download/v2.12.2/mavsdk_2.12.2_ubuntu20.04_amd64.deb
sudo dpkg -i mavsdk_2.12.2_ubuntu20.04_amd64.deb

# Method 2: Build from source
git clone https://github.com/mavlink/MAVSDK.git --recursive
cd MAVSDK && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install && sudo ldconfig

# Install PX4 for simulation
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo
```

### Running Simulation

```bash
# Terminal 1: Start Gazebo simulation
gazebo --verbose worlds/iris.world

# Terminal 2: Run surveillance system
./bin/drone_surveillance
```

## Project Structure

```
drone-surveillance-system/
├── src/
│   ├── flight_control/     # MAVSDK integration and flight logic
│   ├── computer_vision/    # Object detection and tracking
│   ├── ui/                # Ground control station
│   └── utils/             # Utility functions
├── include/               # Header files
├── tests/                # Unit and integration tests
├── config/               # Configuration files
├── docs/                 # Documentation and diagrams
├── scripts/              # Build and deployment scripts
└── data/                 # Test data and recordings
```

## 🎯 Development Roadmap

### Phase 1: Core Flight Control ✅
- [x] MAVSDK integration
- [x] Basic takeoff/landing
- [ ] Waypoint navigation
- [ ] Failsafe mechanisms

### Phase 2: Video System
- [ ] Camera integration
- [ ] Live video streaming
- [ ] Recording functionality

### Phase 3: Computer Vision
- [ ] Object detection
- [ ] Tracking algorithms
- [ ] Alert system

### Phase 4: Ground Station
- [ ] Real-time monitoring UI
- [ ] Flight path planning
- [ ] Data analysis tools

## Testing

```bash
# Run unit tests
cd build
make test

# Run integration tests
./tests/integration_test
```

## Documentation

- [API Documentation](docs/api.md)
- [User Guide](docs/user_guide.md)
- [Development Guide](docs/development.md)
- [Hardware Setup](docs/hardware.md)

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## 🆘 Support

- Create an issue for bugs/features
- Check existing documentation
- Contact: [lachlankelliher@gmail.com]

---

**Status**: 🚧 In Development | **Version**: 0.1.0 | **Last Updated**: August 2025