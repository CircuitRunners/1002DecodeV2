# AI Coding Agent Guidelines for 1002DecodeV2 FTC Project

## Project Overview
This is an FTC (FIRST Tech Challenge) robot control application based on the official FTC SDK v11.0. The project uses a multi-module Gradle structure with Android Studio, targeting Java 8 and Android API 28.

## Architecture
- **FtcRobotController**: Library module containing the FTC SDK framework, samples, and robot controller logic.
- **TeamCode**: Application module for team-specific OpModes, subsystems, and configurations.
- OpModes are annotated Java classes that run robot code, either teleoperated (@TeleOp) or autonomous (@Autonomous).

## Key Files and Directories
- `build.common.gradle`: Shared build configuration for the app module.
- `build.dependencies.gradle`: Centralized dependency management.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`: User code root.
  - `OpMode/`: Robot operation modes (teleop and auto).
  - `Config/`: Hardware configurations and subsystems.
  - `Testers/`: Diagnostic and testing utilities.
- `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/`: Official sample OpModes.

## Development Workflows
- **Building**: Use `./gradlew build` or Android Studio's build tools.
- **Deployment**: Build APK and sideload to robot controller, or use Android Studio's run/debug.
- **Testing**: Use `Testers/` OpModes for hardware validation (e.g., SensorTimingTester for I/O performance).
- **Debugging**: Leverage `telemetry` for real-time data and `RobotLog` for persistent logging. Search logs with tags like "SensorTiming".

## Coding Patterns
- **OpMode Structure**: Initialize hardware in `init()`, run loops in `loop()` or `runOpMode()`. Use `ElapsedTime` for timing.
- **Hardware Access**: Retrieve devices via `hardwareMap.get()` with config names (e.g., "fl" for front-left motor).
- **Efficiency**: Enable bulk reads for motor encoders to reduce I2C latency. Cache sensor data when possible.
- **Annotations**: Use `@Configurable` from fullpanels for runtime tuning. Disable OpModes with `@Disabled`.
- **Subsystems**: Encapsulate hardware groups (e.g., `Sensors` class for color sensors) for modularity.
- **Dependencies**: Integrate third-party libraries like PedroPathing for advanced pathing, GoBilda Pinpoint for odometry.

## Conventions
- **Sample Naming**: Follow FTC prefixes (Basic, Sensor, Robot, Concept, Utility) for clarity.
- **Device Names**: Standardize config names (e.g., "left_drive", "sensor_color") matching sample conventions.
- **Code Style**: Adhere to Google Java Style Guide. Use meaningful variable names derived from config names (e.g., `driveFL` from "fl").
- **Error Handling**: Check for disconnections (e.g., `isHubDisconnected()`) in loops.
- **Versioning**: App version synced with FtcRobotController's AndroidManifest via Groovy script in build.common.gradle.

## External Integrations
- **FTC SDK**: Core framework for robot control, vision, and hardware abstraction.
- **PedroPathing**: Advanced path following and localization.
- **FTCDashboard**: Real-time telemetry and configuration.
- **FullPanels**: Runtime parameter adjustment via annotations.
- **SolversLib**: Mathematical utilities for kinematics and control.

## SRSHub Integration
The `Sensors` class encapsulates all interactions with the SRSHub device, including initialization, configuration of I2C devices (e.g., APDS9151 color sensors), and data retrieval.

Reference `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/sample_conventions.md` for detailed sample guidelines.</content>
<parameter name="filePath">/Users/connorreddington/AndroidStudioProjects/1002DecodeV2/AGENTS.md
