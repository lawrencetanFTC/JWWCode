# FTC Robotics Team Codebase

Welcome to the code repository of **19897 JWW**, participating in the FIRST Tech Challenge (FTC). This project includes all code necessary for the TeleOp and Autonomous phases of the competition, utilizing advanced technologies like OpenCV for color detection and RoadRunner for motion planning. Our robot is designed for precision and efficiency, aiming to excel in every aspect of the competition.

## Project Overview
This repository contains the code for our FTC robotics team. We have implemented both TeleOp and Autonomous programs for controlling the robot during different phases of the competition. Additionally, we use OpenCV for color detection, allowing the robot to react to color-based tasks automatically.

## Installation and Setup
Follow the instructions below to set up the development environment and run the programs.

### Prerequisites
* Java SDK (8 or later)
* Android Studio: To program and deploy code to the FTC robot controller.
* FTC SDK: Download from the official FTC GitHub repository.
* RoadRunner Library (for motion planning): Follow the instructions from the official RoadRunner documentation.

#### System Requirements:
* Operating System: Windows 10 / macOS 10.14 or later
* Android Device: Requires a device running Android 7.0 (Nougat) or higher
* Minimum Storage: 500 MB of free space
* Internet: For syncing with GitHub and downloading dependencies
#### OpenCV Version
* OpenCV Version: 4.5.3 or higher. Ensure that you have this version installed in your Android Studio project dependencies.

## Steps
Clone this repository to your local machine.
```
git clone https://github.com/lawrencetanFTC/JWWCode.git
```
1. Open the project in Android Studio.
2. Go to File > Open and select the project folder.
3. Sync the project with the FTC SDK by going to Build > Sync Project with Gradle Files.
4. Connect your Android phone (robot controller) and deploy the code using Run > Run ‘FtcRobotController’.
5. Install the Driver Station app on your driver phone (available on Google Play Store).

## Code Structure
The code is divided into several programs, each serving different phases of the competition. Below are descriptions of each program:

**TeleOp**: WorkTeleOp
This is the TeleOp code that allows the drivers to control the robot manually during the TeleOp phase of the competition.

**Mecanum Drive**: Implements a mecanum wheel system for omnidirectional movement.
### Mechanisms:
**Spin Take**: Continuous servo for intaking game elements.
**Rack and Pinion Servo**: For extending mechanisms.
**Arm Movement**: Motor for controlling arm movement.
**Claw Control**: Servo to open and close the claw for gripping game elements.

### Key Controls:
**Gamepad 1:**
* Left stick for moving (forward, backward, and strafe).
* Right stick for turning.
* Bumpers for arm control.
* Buttons for activating spin take and rack and pinion mechanisms.
#### Gamepad 2:
* Bumpers to open/close the claw.
* Joysticks for controlling additional side mechanisms.
  

### Autonomous: DeckAuto
This autonomous program uses the RoadRunner library for precise movement and waypoint-based navigation during the autonomous phase.

**Waypoints:**
* Waypoint 1: Moves from the starting position to deposit a specimen.
* Waypoint 2: Moves towards sample collection.
* Waypoint 3: Pushes the samples to the observation zone.
* Waypoint 4: Moves to the landing zone for the end of the autonomous phase.

### Autonomous: BasketAuto
Similar to DeckAuto, this program also controls the robot autonomously but focuses on handling basket-based tasks during the autonomous period.

**Waypoints:**
* Waypoint 1: Moves to drop off a specimen.
* Waypoint 2: Moves to collect samples.
* Waypoint 3: Places the elements in a high basket.
* Waypoint 4: Navigates to the landing zone.

**PropDetection:** ColorBasedMoveWithDistance
This program uses OpenCV for color detection and movement based on color recognition.
**Webcam:** A camera captures live feed to detect the colors red and blue.
**Color Detection:**
* If blue is detected, the robot moves forward.
* If red is detected, the robot moves backward.
**Encoders**: Motor encoders are used to control the movement distance.

**Key Features:**
Custom OpenCV pipeline to filter blue and red colors.
Precise motor control based on encoder counts to move the robot in small increments.

For a detailed look at all files, visit the [directory](JWWCode/road-runner-quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode).

### Breif MecanumDrive Class Overview

This class implements a mecanum drive system for an FTC robot, including motor control, localization, and trajectory following.

#### Components:
1. **Params**: 
   - Configurable parameters for the mecanum drive, such as IMU orientation, feedforward constants, path/turn profile parameters, and controller gains.

2. **Kinematics**: 
   - Uses `MecanumKinematics` to convert between wheel velocities and robot movement (forward, lateral, and rotation).

3. **Motors**: 
   - Four `DcMotorEx` motors: `leftFront`, `leftBack`, `rightBack`, `rightFront`.
   - These control the four mecanum wheels.

4. **IMU**:
   - Uses `LazyImu` for obtaining heading information from the robot’s Inertial Measurement Unit (IMU).

5. **Localizer**:
   - The `DriveLocalizer` class calculates the robot’s position (pose) based on wheel encoder readings and IMU data.

#### Key Methods:

1. **setDrivePowers()**: 
   - Sets the power for each wheel based on the robot's desired movement in 2D space.

2. **updatePoseEstimate()**: 
   - Updates the robot's current position and velocity based on sensor data.

3. **Trajectory Following**:
   - `FollowTrajectoryAction`: Executes a time-based trajectory to move the robot along a path.
   - `TurnAction`: Rotates the robot to a specific angle.

4. **Visualization**:
   - Uses the `Canvas` class to draw the robot’s estimated pose and trajectory on the dashboard.

#### Example Usage:
- The `MecanumDrive` can be initialized with hardware from the `HardwareMap` and configured using the `PARAMS`.
- Trajectories can be followed using predefined paths, and the robot's movement is adjusted with feedback from the IMU and encoders.

### About `DriveConstants.java`

**The `DriveConstants.java` file is crucial for defining key constants and configurations that your robot uses for motion and control. Here's is what it contains:**
#### 1. **Motor Control** 
- **Constants like `TICKS_PER_REV`, `MAX_RPM`, and `MOTOR_VELO_PID`**: These are used to control motor behavior, especially when managing velocity through encoders.
- **`RUN_USING_ENCODER`**: This flag determines if you're using encoder feedback for precise motor control, which is critical for accurate movement.

#### 2. **Feedforward and Motion Parameters** 
- **Parameters like `kV`, `kA`, and `kStatic`**: These constants model how the motors respond to commands. They are essential for trajectory generation and motion control, especially when using tools like RoadRunner.

#### 3. **Physical Robot Parameters** 
- **Constants like `WHEEL_RADIUS`, `TRACK_WIDTH`, and `GEAR_RATIO`**: These define your robot's physical dimensions. These values are critical for accurate distance and angular calculations during movement.
- These constants impact both autonomous and manual (TeleOp) robot control.

#### 4. **Trajectory Generation** 
- **Constants like `MAX_VEL`, `MAX_ACCEL`, `MAX_ANG_VEL`, and `MAX_ANG_ACCEL`**: These are used by the RoadRunner library for planning paths and ensuring the robot moves within safe, physical limits.

#### 5. **Robot Orientation**
- **`LOGO_FACING_DIR` and `USB_FACING_DIR`**: These define how your robot's control hub is mounted, helping the SDK and RoadRunner library determine the correct reference frame for orientation tracking.

#### 6. **Encoders and Velocity Calculations**
- **Functions like `encoderTicksToInches` and `rpmToVelocity`**: These convert encoder ticks and motor RPMs into practical units (like inches), which are crucial for both feedback control and motion planning.

---

- If you're using **RoadRunner for motion planning**, this file ensures that your robot moves accurately according to its mechanical design.
- It **centralizes important constants**, making it easier to update and tune the robot without modifying multiple files.
- These values affect both **autonomous movement** (via trajectory planning) and possibly **manual control** (if using encoder feedback in TeleOp).

### What to Do in this file
1. **Tune the Constants**: The comments suggest some constants (e.g., `MOTOR_VELO_PID`, `kV`, `kA`) need tuning. This is essential for optimal robot performance.
2. **Review Physical Constants**: Make sure measurements like **wheel radius** and **track width** match your robot's actual dimensions.
3. **Dashboard Use**: You can adjust certain parameters using the **FTC Dashboard**, and this file allows you to make those changes persist in your code.

### Two-Wheel Localizer

Our robot uses a **two-wheel localizer** for tracking position on the field. The localizer consists of two dead wheels (parallel and perpendicular), which provide accurate positional feedback during autonomous operations. This approach allows the robot to maintain precise localization without relying on odometry from the drive motors.

#### How It Works
- The **parallel encoder** measures the robot’s forward/backward movement.
- The **perpendicular encoder** measures the robot’s lateral (side-to-side) movement.
- An **IMU (Inertial Measurement Unit)** is used to track the robot's heading (rotation).

By combining data from both encoders and the IMU, we can accurately calculate the robot’s position and orientation during autonomous motion. This data is used to control the robot’s movement and trajectory in real time.

#### Key Features
- **Encoders:** The two dead wheels are attached to encoders that track movement independently of the drive motors.
- **IMU Integration:** The IMU tracks the robot's heading, and this information is combined with encoder data for precise localization.
- **Flight Recorder:** The system logs key data (encoder values, heading) for debugging and fine-tuning.

#### Code Implementation
We use the `TwoDeadWheelLocalizer` class to implement the two-wheel localization system. Here are some key components:

- `par` (parallel encoder) and `perp` (perpendicular encoder): These measure the robot's movement in respective directions.
- `imu`: Tracks the robot's orientation.
- The `update()` method combines encoder and IMU data to provide a **twist** representing the robot's motion (translation and rotation).

#### Benefits of Two-Wheel Localizer
- **Accurate Position Tracking:** The two-wheel setup allows for more precise localization compared to single encoder-based systems, especially when combined with the IMU.
- **Independent of Drive Motors:** Since the encoders are not attached to the drive motors, the localization is not affected by wheel slippage or drivetrain inconsistencies.

This localization method is especially useful for our **Autonomous** programs (`DeckAuto`, `BasketAuto`), where precise movement and positioning are crucial for task completion.


## Usage
To run any of the programs:

1. Open the Driver Station app and pair it with the Robot Controller.
2. Choose between TeleOp or Autonomous modes.
3. Follow the controls as described in the code structure.
### TeleOp Mode:
* Select WorkTeleOp from the TeleOp options.
### Autonomous Mode:
* Select DeckAuto, BasketAuto, or ColorBasedMoveWithDistance based on the task at hand.

## Contributing
#### How to Contribute
We welcome contributions to improve the robot’s functionality, optimize code, or add new features. Here's how you can contribute:

1. **Fork this repository** by clicking on the "Fork" button at the top of this page.
2. **Clone** your fork to your local machine:
   ```bash
   git clone https://github.com/your-username/repository-name.git
3. Create a new branch for your new feature of bugfix:
   ```bash
   git checkout -b my-feature-branch
4. Make changes and commit your work:
   ```bash
   git commit -m "Added new feature or bugfix"
5. Push to your fork and submit a pull request:
   ```bash
   git push origin my-feature-branch

### 4. **Tests**
If your code includes automated tests, add a section that explains how to run the tests:

## Running Tests
To ensure that everything is working correctly, we have included unit tests for the main modules.

### Running Tests in Android Studio:
1. Open Android Studio and load the project.
2. Navigate to `src/test/java/org/firstinspires/ftc/teamcode/tests/`.
3. Right-click on a test file and choose **Run**.

Make sure to test all the relevant OpModes before deployment.


## Creating Your Own OpModes

The easiest way to create your own OpMode is to use the structure of one of our existing OpModes and modify it for your needs. We provide examples of **TeleOp** and **Autonomous** modes to get you started.

The key OpModes in this repository are:
- `WorkTeleOp`: For TeleOp control of the robot.
- `DeckAuto`: Autonomous program for navigating a specific field setup.
- `BasketAuto`: Autonomous program for navigating another field configuration.
- `ColorBasedMoveWithDistance`: Uses OpenCV to detect colors and adjust movement based on distance.

### Naming Conventions

We follow a standard naming convention to keep the code organized and easy to understand. Below is the convention used for the class names:

- **TeleOp**: Programs that allow the driver to control the robot manually.
- **Auto**: Programs that run autonomously during the initial phase of the match.
- **PropDetection**: Code related to object or color detection.

For example:
- `WorkTeleOp`: Represents a TeleOp program for manual control.
- `DeckAuto`: Represents an autonomous program designed for a specific field setup.
- `ColorBasedMoveWithDistance`: Detects color properties and moves based on the detected color's distance.

## Modifying OpModes

To modify an OpMode or create your own, follow these steps:

1. Locate the desired OpMode in the `src/main/java/org/firstinspires/ftc/teamcode` folder.
2. Copy the OpMode and rename it according to the new functionality you wish to add.
3. Modify the functionality by editing the program logic inside the class.
4. Adjust the following lines in your new OpMode:
   ```java
   @TeleOp(name="Your Custom OpMode Name", group="TeleOp/Auto")
   @Disabled

## Troubleshooting/FAQ
### Common Issues
- **Gradle sync failed:** Ensure you have the correct version of the FTC SDK and that all dependencies are installed.
- **Robot Controller app crashes:** Check that you have properly deployed the latest version of the app, and ensure no conflicting OpModes are running.
- **RoadRunner not functioning properly:** Verify that the RoadRunner paths are correctly set up in the configuration file.

If you encounter other issues, please open a [GitHub Issue](https://github.com/your-repo/issues).

## Changelog
#### [v1.1.0] - 2024-10-16
- Added ReadMe
#### [v1.0.0] - 2024-10-17
- 

## Credits
- FTC SDK: [FTC GitHub Repository](https://github.com/FIRST-Tech-Challenge)
- RoadRunner Library: [RoadRunner Documentation](https://learnroadrunner.com)
- OpenCV: [OpenCV Official Site](https://opencv.org)

Special thanks to all the team members who contributed to this project!
- **Programming Team Members for meet 1**:
  - Lawrence (Lead Programmer)
  - Shreyansh 
  - Arnav
  - Sohan(Also outreach)
  - Likith(Also engineering)
  - Samvarth(Also engineering)
  - **Programming Team Members on reserve**
  - Omar
  - Pratham


