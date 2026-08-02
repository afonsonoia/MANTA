# MANTA – An Embedded AI Platform for Vision-Based Autonomous Flight

![MANTA Project](media/general_images/main_image.png)

**MANTA is an experimental fixed-wing UAV research platform designed to investigate embedded computer vision and autonomous navigation in GPS-denied environments.**

## Why MANTA?

**Rather than being a commercial autopilot, MANTA is intended as an experimental platform for developing, testing and validating embedded perception and autonomous navigation algorithms under real flight conditions.**

**🚧 Current Development:** Preparing the aircraft for its first manual flight tests.

> *Fun Fact: The name **MANTA** is a regional term from Madeira Island, Portugal. It refers to the local population of the Common Buzzard (*Buteo buteo rothschildi*), a resident subspecies frequently seen soaring above the island's mountains.*

**Living Document:** This roadmap represents the current plan but is subject to change at any time as the project evolves, new challenges arise, or testing results dictate new priorities.

---

## Project Overview

The aircraft is powered by a hybrid architecture combining a low-level microcontroller for flight control and a companion computer for high-level intelligence and computer vision:

*   **Flight Controller (ESP32)**: Handles real-time tasks including sensor data acquisition, radio control (RC) receiver channel decoding (PWM/PPM), telemetry, and servo/ESC actuator outputs (PID control loops for elevator and roll stabilization).
*   **Companion Computer (Raspberry Pi 3 A+)**: Hosts the camera and executes lightweight computer vision pipelines for visual place recognition, estimating the aircraft's position by matching real-time imagery against a geo-referenced aerial database.
*   **Sensor Suite**:
    *   **GPS**: Primary navigation reference (when signal is active).
    *   **IMU (MPU6050)**: Accelerometer and Gyroscope for attitude estimation.
    *   **Barometer (BMP280)**: Altitude holding and barometric tracking.
    *   **Voltage Sensor**: Real-time battery cell monitoring.
    *   **Camera**: Captures ground features for visual localization.
*   **Custom PCBs (Shield Design)**: A custom two-board shield architecture designed in KiCad. This approach reduces manufacturing costs and results in a more compact footprint that easily fits within the airframe. It cleanly distributes power and interconnects the ESP32, Raspberry Pi, LoRa radio, sensor breakout boards, receiver, and servos. The design incorporates extensive GND copper pours around sensor zones, providing improved heat dissipation and reduced electromagnetic interference around sensitive electronics.

---

## Repository Structure

The project is structured as follows:

```
├── Code/
│   ├── Battery_Monitor/            # Raw battery voltage sensor data logger & ESC throttle control GUI
│   └── Computer Vision/
│       ├── demos/
│       │   └── MobileNet + KNN/        # Visual localization, image recognition & database loaders
│       ├── general_examples/           # ESP32 POCs (LoRa, Barometer, Receiver, Throttle/Voltage telemetry)
│       ├── Simulator/                  # Python-based flight simulator and autopilot PID tuning GUI
│       └── Test_2/Test_10_02_2026/     # Main low-level ESP32 test sketch (servo, battery, receiver input)
└── Electronics/
    ├── MANTA.kicad_sch                 # Custom PCB Schematic
    ├── MANTA.kicad_pcb                 # Custom PCB Layout
    └── MANTA.kicad_pro                 # KiCad Project configuration
```

---

## Current Project Status

*   **Low-Level Firmware**: ESP32 test sketch successfully integrates PWM receiver input reading, deadband smoothing, battery percentage estimation, and servo/ESC command output.
*   **Voltage Sensor Calibration**: Experimentally determined the conversion function for raw ADC voltage sensor data in the ESP32 firmware, enabling accurate battery voltage measurements.
*   **Initial LoRa Communication**: Established initial bidirectional LoRa communication between the aircraft (MANTA ESP32) and the ground station, streaming live battery telemetry and RSSI/SNR signal metrics while receiving ESC control commands.
*   **Visual Navigation**: Python POCs demonstrate lightweight deep feature extraction combined with visual place recognition to predict coordinates based on matching signatures.
*   **Electronics**: The custom PCBs (utilizing a compact 2-board shield architecture) have been manufactured and assembled. We are currently finalizing the electrical integration on the airframe.
*   **Simulation**: Basic PyQt-based simulation setup created for autopilot PID tuning loops.
*   **Next Steps**: Complete avionics mounting and prepare for maiden manual flight tests.

### PCB Previews

<p align="center">
  <img src="media/general_images/Kicad_pcbs/power.png" width="48%" alt="PCB Power Routing">
  <img src="media/general_images/Kicad_pcbs/data.png" width="48%" alt="PCB Data Routing">
</p>

---

## Project Roadmap

### Phase 1: Hardware Proof-of-Concepts (POC)
- [x] Establish ESP32 communications with peripheral sensors (BMP280, LoRa, Voltage sensor).
- [x] Implement RC receiver pulse width reading and telemetry logs.
- [x] Establish initial LoRa communication between aircraft and ground station.
- [x] Set up lightweight visual place recognition experiments using aerial imagery databases.
- [x] Perform isolated bench tests for each sensor to ensure accuracy and data reliability.

### Phase 2: Airframe Assembly & Avionics Integration
- [x] Finish assembling the main airframe skeleton from the purchased kit.
- [x] Complete custom KiCad PCB schematic (`MANTA.kicad_sch`).
- [x] Finalize PCB routing, trace widths for power distribution, and manufacture the board.
- [x] Assemble the PCB and perform physical continuity and power testing.
- [x] Experimentally calibrate the raw voltage sensor conversion function to obtain accurate voltage readings from raw ADC data.
- [x] Experimentally study and map the battery discharge curve under motor load using the calibrated voltage sensor.
- [ ] Design and build a sturdy, custom mount for the Raspberry Pi camera module.
- [ ] Mount avionics (ESP32, Raspberry Pi, camera, sensors) on the airframe.
- [ ] Conduct EMI (Electromagnetic Interference) testing to ensure motors/ESCs don't disrupt the GPS or LoRa signals.
- [ ] Execute maiden field test in manual flight mode to validate airframe aerodynamics and real-time ground station telemetry streaming.

### Phase 3: Software, Autopilot & Path-Planning
- [ ] Integrate IMU (MPU6050) Kalman/Complementary filtering into low-level ESP32 firmware.
- [ ] Implement robust pitch (elevator) and roll (aileron) stabilization PIDs.
- [ ] Perform static PID tuning on a test rig (without flying) to evaluate control surface responsiveness.
- [ ] Set up ESP32-to-Raspberry Pi serial communication protocol (UART/MAVLink-like packets).
- [ ] Implement high-level LoRa telemetry control (e.g., "go to waypoint X").
- [ ] Develop and integrate an autonomous path-planning algorithm.

### Phase 4: Ground Station & Virtual FPV
- [ ] Implement a basic Virtual FPV on the ground station (using telemetry, altitude, coordinates, and 3D maps) to artificially generate the pilot's view for terrain avoidance.
- [ ] Enhance Virtual FPV by streaming additional camera/sensor data to realistically detect and display obstacles like trees.
- [ ] Conduct field tests to evaluate LoRa telemetry maximum range, packet loss, and Virtual FPV rendering latency.

### Long-Term Research Directions
- Visual-Inertial State Estimation
- Lightweight onboard perception
- Embedded neural network optimization
- Sensor fusion for robust navigation
- Dataset acquisition and evaluation
- Fully autonomous visual flight in GNSS-degraded environments
