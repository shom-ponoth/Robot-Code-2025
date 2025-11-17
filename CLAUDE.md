# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Team 6560 FRC 2025 Robot Codebase Architecture

## Common Development Commands

### Building and Deployment
- `./gradlew build` - Compile the robot code and run tests
- `./gradlew deploy` - Build and deploy code to the RoboRIO (requires connection to robot network)
- `./gradlew` - Download Gradle wrapper and all vendor dependencies (run this first on fresh clone)

### Testing
- `./gradlew test` - Run all JUnit 5 tests
- Tests are located in `src/test/java/` and use JUnit Platform

### Simulation
- `./gradlew simulateJava` - Launch robot simulation with GUI (includes DriverStation)
- Simulation uses WPILib's built-in GUI for testing without hardware
- CTRE Phoenix 6 devices simulate in `ctre_sim/` directory

### Cleaning
- `./gradlew clean` - Remove all build artifacts

## Overview
This is a Java-based FRC robot project for the 2025 season. The robot uses a **swerve drivetrain** with **YAGSL (Yet Another Generic Swerve Library)** integration and **PathPlanner** for autonomous routines. The robot has a complex mechanism system with elevator, wrist, intake grabbers, and climbing functionality.

## High-Level Architecture

### 1. Robot Code Structure

#### Entry Point
- **`Main.java`**: Simple entry point that starts the Robot class via `RobotBase.startRobot()`
- **`Robot.java`**: Main robot class extending `TimedRobot`
  - Manages robot lifecycle (init, periodic, autonomous, teleop, disabled modes)
  - Instantiates and schedules commands via `CommandScheduler`
  - Handles motor brake state management during disabled periods
  - Implements motor brake locking for 10 seconds after disable to hold robot position

#### Robot Configuration
- **`RobotContainer.java`**: Central configuration hub for the entire robot
  - Creates all subsystems (drivebase, climb, elevator, wrist, ball/pipe grabbers)
  - Sets up all controller bindings (Xbox controllers)
  - Creates and configures the autonomous chooser (SmartDashboard)
  - Registers PathPlanner named commands for use in autonomous
  - Configures default commands for all subsystems
  - Sets up swerve drive input stream with custom joystick processing (deadbands, scaling)

- **`Constants.java`**: All robot tuning constants and configuration values
  - `DrivebaseConstants`: Wheel lock timing
  - `OperatorConstants`: Joystick deadbands
  - `ElevatorConstants`: Motor IDs, PID values, elevator heights (STOW, L2, L3, L4)
  - `WristConstants`: Motor IDs, encoder offsets, position bounds, wrist angles
  - `ClimbConstants`: Motor IDs, position bounds, gear ratios

- **`ManualControls.java`**: Maps Xbox controller inputs to command-level actions
  - Uses two Xbox controllers (driver + operator)
  - Provides boolean queries for elevator states (goToL1, goToL2, etc.)
  - Provides queries for climb, intake, outtake, wrist positioning
  - Includes deadband and axis processing utilities

## 2. Swerve Drive Integration (YAGSL)

### SwerveSubsystem Architecture
**Location**: `/src/main/java/com/team6560/frc2025/subsystems/swervedrive/SwerveSubsystem.java` (809 lines)

The `SwerveSubsystem` is the largest subsystem and contains:

#### Initialization
- **YAGSL SwerveParser**: Loads swerve configuration from JSON files
- Configuration directory: `src/main/deploy/swerve/falcon/`
  - `swervedrive.json`: Main swerve config (IMU: Pigeon2 on Canivore, 4 modules)
  - `modules/frontleft.json`, `modules/frontright.json`, `modules/backleft.json`, `modules/backright.json`:
    - Each specifies drive motor (Kraken X60), angle motor (Spark Max), encoder (CANcoder)
    - Sets absolute encoder offsets and inversion flags
  - `modules/pidfproperties.json`: PID tuning
  - `modules/physicalproperties.json`: Gear ratios and physical specs
  - `controllerproperties.json`: Controller PID values

#### Core Functionality
1. **PathPlanner Integration** (`setupPathPlanner()` method)
   - Uses `AutoBuilder` to configure path following
   - Translation PID: kP=5.3
   - Rotation PID: kP=4.0
   - Implements custom drive consumer that accepts ChassisSpeeds
   - Uses `PPHolonomicDriveController` for holonomic path tracking
   - Supports alliance-relative path mirroring for red/blue sides
   - Pre-warms PathFinder on initialization

2. **Autonomous Commands** 
   - `getAutonomousCommand(String pathName)`: Returns a `PathPlannerAuto` command
   - `driveToPose(Pose2d pose)`: Uses PathPlanner pathfinding to drive to arbitrary pose
   - `driveToNearestPoseLeft()` / `driveToNearestPoseRight()`: Drive to nearest scoring location
   - Target pose lists are hardcoded for left and right alliance positions (12 positions each side)

3. **Vision Integration** (Limelight with MegaTag2)
   - **Periodic method**: Continuously pulls vision data from Limelight via `LimelightHelpers`
   - Uses `getBotPoseEstimate_wpiBlue_MegaTag2()` for pose estimation
   - Adds vision measurements to odometry with latency compensation
   - Vision standard deviations: 0.08m xy, 2 radians rotation
   - Updates robot orientation for Limelight each cycle

4. **Telemetry & Debugging**
   - High verbosity telemetry enabled
   - Field visualization with target pose markers
   - SmartDashboard integration for real-time tuning

## 3. Mechanism Subsystems

### Subsystem Pattern
All mechanism subsystems extend `SubsystemBase` and follow similar patterns:
- Use CTRE Phoenix6 TalonFX motors with Canivore CAN bus
- Update NetworkTables for dashboard visibility
- Implement state machine patterns where applicable

### Key Mechanism Subsystems

#### Elevator (`Elevator.java`)
- **Motors**: Two TalonFX motors (IDs 14, 15) linked for redundancy
- **Sensors**: Upper and lower limit switches
- **States**: STOW, L2, L3, L4 (for normal scoring) + shifted variants (S_L2, S_L3, S_L4)
- **Heights**: Heights are motor rotations (STOW=0.4, L4=17.65)
- **Control**: Position control via `PositionVoltage` with PID (kP=0.7, kG=0.4 for gravity compensation)
- **Safety**: 40A current limit, brake mode active

#### Wrist (`Wrist.java`)
- **Motor**: Single TalonFX (ID 16)
- **Sensor**: CANcoder absolute encoder (ID 17) for position feedback
- **States**: STOW, PICKUP, L1, L2, L4 (and shifted variants)
- **Angle Bounds**: -5 to 254 degrees with soft limits
- **Control**: Position control with 108:1 gear ratio
- **Features**: Integrated limit switch for homing

#### Ball Grabber (`BallGrabber.java`)
- **Motor**: Single Spark Max brushless (ID 25)
- **Operation**: Current-sensing based intake (stops at 30A current)
- **Speeds**: Intake at -0.3, Outtake at 0.7 (duty cycle)

#### Pipe Grabber (`PipeGrabber.java`)
- Similar structure to Ball Grabber
- Used for scoring/intaking game pieces

#### Climb (`Climb.java`)
- **Motors**: Two TalonFX motors (IDs 20, 21) with reverse linkage
- **Sensor**: CANcoder absolute encoder (ID 22)
- **Soft Limits**: -0.162 to 0.198 rotations
- **Control**: Manual operation via operator joystick with percentage-based commands
- **Gear Ratio**: 1.5:1

### Command Pattern for Mechanisms

All mechanism control is handled through dedicated **Command** classes that:
1. Read inputs from `ManualControls`
2. Map controller inputs to subsystem states/setpoints
3. Are set as default commands (run continuously during teleop)

#### Example: ElevatorCommand
- Maps ABXY buttons to elevator levels (A=L1, X=L2, B=L3, Y=L4)
- Right bumper enables "shifted" states with alternate heights
- Implements state machine within the command's execute() method
- Called every 20ms by CommandScheduler

Similar pattern for: `WristCommand`, `ClimbCommand`, `BallGrabberCommand`, `PipeGrabberCommand`

## 4. Autonomous Routines

### PathPlanner Integration

**Configuration Files**: `src/main/deploy/pathplanner/`

#### Settings
- Swerve module dimensions and wheel radius configured
- Max velocity: 5.0 m/s, Max accel: 3.75 m/s²
- Max angular velocity: 540°/s

#### Path Format
- Paths stored as `.path` files (human-readable JSON)
- Autos stored as `.auto` files defining sequential command chains
- Example: `Taxi Auto.auto` contains a sequential list referencing path `taxiSeg1`

#### Auto Routines (in RobotContainer)

The robot has multiple autonomous modes selectable via SmartDashboard:

1. **1p Mid Auto** (Default)
   - Loads PathPlanner path `Taxi Auto` 
   - Followed by `ScoringL4` command

2. **Aero 3 Processor Auto**
   - Multi-segment routine: 6 path segments chained with scoring/intake commands
   - Paths: `Aero3pSeg1p` → Score → `Aero3pSeg2p` → Intake → ... → `Aero3pSeg6p`

3. **Aero 3 No Processor Auto**
   - Variant without processor interaction: 5 segments with 3 scoring runs

4. **Bump Auto**
   - Alternative path routing using `AeroBump-4`, `AeroBump-5` segments

5. **Hue 2.5 Auto**, **Auto Align Test**, **Score Test**, **Taxi**

### Named Commands Registration
PathPlanner auto segments can reference "named commands" registered in `RobotContainer`:
- `"Scoring L4"` → `ScoringL4` command
- `"Station Intake"` → `StationIntake` command  
- `"TravelingL3"` → `L3Travel` command

### Scoring L4 Auto Command (`ScoringL4.java`)
This is the primary scoring mechanism used in multiple autos:

**Sequence**:
1. **MechanismUp**: Raises elevator to L4 height and wrist to L4 angle in parallel
2. **EjectPiece**: Waits 0.2s then runs grabber at max speed for 0.4s total
3. **MechanismDown**: Lowers wrist to PICKUP position, then after 0.25s lowers elevator to STOW

Uses `FunctionalCommand` with termination conditions based on encoder positions

## 5. Vision & Localization

### Limelight Integration
- **Camera Type**: Limelight with MegaTag2 april tag detection
- **Vision Pipelines**: Three custom pipelines configured
  - `teleop.vpr`: General purpose detection for teleop
  - `auto_blue.vpr`, `auto_red.vpr`: Alliance-specific pipelines
  - Pipelines configured for AprilTag detection with detailed tuning (exposure, balance, etc.)

### Odometry Update Pipeline
1. Every periodic cycle (20ms), SwerveSubsystem queries Limelight
2. Gets pose estimate with timestamp and tag count
3. If ≥1 tags detected, adds vision measurement to swerve odometry
4. Uses `LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2()`
5. Compensates for network latency

### Pose Tracking
- Robot relative pose maintained by YAGSL swerve drive odometry
- Vision provides periodic corrections
- Field visualization on SmartDashboard shows robot pose and target positions

## 6. Input & Control System

### Controller Setup
- **Driver Controller**: Xbox 0 (main drive control)
  - Left stick: Translation X/Y with deadband 0.1
  - Right stick X: Rotation (squared for finer control)
  - Start: Zero heading (skip AprilTags)
  - B: Reset odometry to Limelight
  - X: Drive to nearest scoring location (left side)
  - A: Drive to nearest scoring location (right side)

- **Operator Controller**: Xbox 1 (mechanism control)
  - ABXY buttons: Elevator height selection
  - Right bumper: Shift to alternate mechanism states
  - Right stick Y: Climb up/down (threshold ±0.7)
  - Triggers/bumpers: Intake/outtake controls

### Joystick Processing (RobotContainer)
- Custom `SwerveInputStream` with:
  - Squared axes for smoother control
  - Dynamic speed reduction (60% of normal) when holding left trigger OR elevator above STOW
  - Alliance-relative control (automatically rotates inputs based on alliance color)
  - Deadband 0.1
  - Translation scaling 0.8

## 7. Configuration & Deployment

### Gradle Build System
- WPILib GradleRIO 2025.2.1
- Java 17 targeting
- Deploys to RoboRIO with fat JAR (all dependencies embedded)
- Files deployed from `src/main/deploy/` to `/home/lvuser/deploy` on RoboRIO

### Vendor Libraries
The project uses several vendor libraries (in `vendordeps/`):
- **PathPlannerLib**: Auto path planning and following
- **Phoenix6**: CTRE motor control (TalonFX, CANcoder)
- **REVLib**: REV Robotics Spark Max motor controllers
- **WPILibNewCommands**: Command framework v2
- **AdvantageKit**: Data logging framework
- **Limelight**: Vision processing

## 8. Key Architectural Patterns

### 1. Command-Based Programming
- All robot behavior is implemented as `Command` objects
- Commands are scheduled via `CommandScheduler` (runs every 20ms)
- Subsystems have default commands that run continuously
- Hierarchical command composition: Sequential/Parallel groups for complex sequences

### 2. Subsystem-Based Organization
- Each mechanism is a separate subsystem with dedicated commands
- Clear separation of concerns (drive, intake, elevator, wrist, climb)
- Subsystems coordinate through shared commands

### 3. Configuration as Code
- Robot tuning constants centralized in `Constants.java`
- Swerve configuration in JSON (loaded at runtime via YAGSL)
- PathPlanner paths/autos in structured JSON format
- Allows field updates without recompiling Java

### 4. Vision-Based Odometry Fusion
- Swerve drive maintains local odometry
- Limelight vision provides global position corrections
- Uses timestamp-based fusion with latency compensation
- Enables reliable autonomous positioning

### 5. State Machine Pattern in Commands
- Mechanism commands read controller state and execute appropriate action
- Example: `ElevatorCommand` checks all controller inputs and sets target position
- Commands have initialize/execute/end phases for setup/teardown

### 6. NetworkTables for Debugging
- Every subsystem logs state to NetworkTables (Shuffleboard)
- Enables real-time tuning and diagnostics on test bench
- No need to redeploy code to see sensor values

## 9. Notable Implementation Details

### Swerve Drive Tuning
- Uses custom YAGSL configuration with:
  - Cosine compensation disabled (simulation compatibility)
  - Angular velocity compensation enabled (0.1 coefficient)
  - Heading correction disabled (better for autonomous)
  - Module encoder auto-sync disabled

### Intake Logic
- **Ball Grabber**: Current-sensing based (stops intaking when stall current reached)
- **Pipe Grabber**: Operator-controlled via button presses

### Climb Implementation
- Range of motion: 125.96° (1.5:1 geared)
- Uses soft limits to prevent damage
- Manual control without automatic sequencing

### Motor Safety
- All TalonFX motors configured with:
  - Brake mode for holding position
  - Current limiting (40A for elevator)
  - Neutral mode = Brake (stops immediately when powered off)

## 10. File Organization Reference

```
src/main/java/com/team6560/frc2025/
├── Robot.java                      # Main robot class
├── RobotContainer.java             # Robot configuration & setup
├── Constants.java                  # All tuning constants
├── Main.java                       # Entry point
├── ManualControls.java             # Controller input mapping
├── subsystems/
│   ├── swervedrive/
│   │   └── SwerveSubsystem.java    # Swerve drive (YAGSL + PathPlanner)
│   ├── Elevator.java               # Elevator mechanism
│   ├── Wrist.java                  # Wrist mechanism
│   ├── BallGrabber.java            # Ball intake/outtake
│   ├── PipeGrabber.java            # Pipe intake/outtake
│   └── Climb.java                  # Climbing mechanism
├── commands/
│   ├── auto/
│   │   ├── ScoringL4.java          # Autonomous scoring sequence
│   │   ├── L3Travel.java           # Travel to L3
│   │   └── [other autos]
│   ├── ElevatorCommand.java        # Teleop elevator control
│   ├── WristCommand.java           # Teleop wrist control
│   ├── ClimbCommand.java           # Teleop climb control
│   ├── BallGrabberCommand.java     # Teleop ball grabber
│   └── PipeGrabberCommand.java     # Teleop pipe grabber
└── utility/
    ├── LimelightHelpers.java       # Vision helper functions
    ├── AlwaysRunCommand.java       # Command that runs when disabled
    └── [other utilities]

src/main/deploy/
├── swerve/falcon/                  # YAGSL swerve configuration
│   ├── swervedrive.json
│   ├── modules/
│   │   ├── frontleft.json
│   │   ├── frontright.json
│   │   ├── backleft.json
│   │   └── backright.json
│   └── controllerproperties.json
└── pathplanner/                    # PathPlanner autonomous paths
    ├── settings.json               # Global PathPlanner config
    ├── paths/                      # .path files (individual segments)
    └── autos/                      # .auto files (full routines)

pipelines/                          # Limelight vision pipelines
├── teleop.vpr
├── auto_blue.vpr
└── auto_red.vpr
```

## 11. Debugging & Development Tips

### SmartDashboard Access
- Auto Chooser for testing different autonomous routines
- Network tables for real-time sensor monitoring (Elevator, Wrist, Climb status)
- Robot pose visualization on Field2d

### Common Tuning
- **Swerve speed**: Modify `Constants.MAX_SPEED`
- **Elevator heights**: Update `ElevatorConstants.ElevatorStates`
- **Wrist positions**: Update `WristConstants.WristStates`
- **Control responsiveness**: Adjust joystick deadbands in `ManualControls`
- **Path following**: Tune PathPlanner PIDs in `SwerveSubsystem.setupPathPlanner()`

### Vision Troubleshooting
- Check Limelight pipeline in `pipelines/` directory
- Monitor Limelight connection in SwerveSubsystem.periodic()
- Verify tag detection count and pose confidence

## Summary

This is a complex, competition-ready FRC robot with:
- Advanced swerve drive with vision integration
- Multi-stage autonomous routines using PathPlanner
- Sophisticated mechanism control (elevator, wrist, grippers, climb)
- Real-time tuning and diagnostics via NetworkTables
- Production-quality error handling and safety measures

The architecture prioritizes modularity, testability, and tunability while maintaining the performance requirements of FRC competition.
