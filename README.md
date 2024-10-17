## Project Overview
This repository contains the code for our FTC robotics team. We have implemented both TeleOp and Autonomous programs for controlling the robot during different phases of the competition. Additionally, we use OpenCV for color detection, allowing the robot to react to color-based tasks automatically.

## Installation and Setup
Follow the instructions below to set up the development environment and run the programs.

### Prerequisites
* Java SDK (8 or later)
* Android Studio: To program and deploy code to the FTC robot controller.
* FTC SDK: Download from the official FTC GitHub repository.
* RoadRunner Library (for motion planning): Follow the instructions from the official RoadRunner documentation.

## Steps
Clone this repository to your local machine.
```
git clone <repository-url>
```
1. Open the project in Android Studio.
2. Go to File > Open and select the project folder.
3. Sync the project with the FTC SDK by going to Build > Sync Project with Gradle Files.
4. Connect your Android phone (robot controller) and deploy the code using Run > Run ‘FtcRobotController’.
5. Install the Driver Station app on your driver phone (available on Google Play Store).

### Code Structure
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


