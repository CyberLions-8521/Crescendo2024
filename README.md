# README
The `post-ocr` branch, if it has not yet been merged with branch `main`, is a current work-in-progress branch being done by the CyberLions to increase readability of the code.  The main contributors of the original code base included the following senior programmers from the class of 2024
* [Jackson Nguyen](https://github.com/jackhnguyen)
* [Rachel Fernandez](https://github.com/PokaDoka1)
* [Kevin Vo](https://github.com/EV3KevinDEV)

Much of the original code base borrows code from three different sources
* WPILib's Command Based Programming framework
* Team 7157 Mubotic's 2023-2024 code (courtesy of [Aaron Yoon](https://github.com/awesomeyooner))
* Team 2710 Jetstream's 2023-2024 code (courtesy of [Kevin Vo](https://github.com/EV3KevinDEV))

For reference for how the 2024 Crescendo game was played, see the associated [Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system).

## Robot Subsystems
The subsystems of the robot include the following
* Drivebase - located in `Drive.java`
* Arm - located in `Joint.java`
* Elevator - located in `Elevator.java`.  This subsystem is lifted by the arm, and can be extended in order to reach the SOURCE or score in the AMP.
* Intake/Shooter - located in `Toaster.java`.  This subsystem is within the elevator, and is what is extended when the elevator extends, and as such, is also lifted by the arm.  This is where the NOTE (2024 game piece) sits after it has been taken in by the robot.
* Intake Guides - located in `Hood.java`.  This subsystem consists of plastic rollers that are mounted above the opening of the intake/shooter subsystem.  These rollers are meant to guide the NOTE into the intake to increase the intake tolerance.
* Intake Guide Joint - located in `HoodWrist.java`.  This subsystem controls the motors that act as a "joint" for the "Hood" subsystem, so that it can flip into place when intake, and flip away from the opening when shooting into the SPEAKER.

## Controls
The robot was controlled using two XBox/Logitech gamepads.  The master controller had the following actions
* Analog Joysticks - controlled the swerve drivebase
* Button A - re-zeroed the turn motors on the swerve drivebase
* Button B - reset the heading of the robot
* Button Back - sets the jog value of the arm/joint to 0.15
* Button Start - sets the jog value of the arm/joint to -0.15

The partner controller had the following actions
* Button Left Bumper - turn on motors of shooter to shoot into the AMP
* Button Right Bumper - turn on motors of shooter to shoot into the SPEAKER
* Button Back - intake
* Button Start - outtake on the rollers
* Button A - raise, extend, and flip robot subsystems into position for the SOURCE and turn on necessary motors to intake NOTES
* Button B - lower, retract, and flip robot subsystems back down to default positions
* Button X - raise, extend, and flip robot subsystems into position for the AMP
* Button Y - raise, extend, and flip robot subsystems for something

## Teleop and Command Based Programming Study
If you are studying the Command Based Programming framework for teleop, then the focus of your study should be on the following files in `src/main/java/frc/robot`.  Note that in any of the subsystem files, the `switch` statements involving subsystem state can be ignored.  The code has been refactored so that it no longer depends on subsystem states.  As of April 2, 2024, the code has yet to be tested on a physical robot.  The code otherwise compiles and builds without error.
* `commands/ElevatorGoToSetpoint.java`
* `commands/HoodWristGoToSetpoint.java`
* `commands/JointGoToSetpoint.java`
* All subsystems within the `subsystems` directory, with the exception of PathHandler, SuperStructure, Tracker, and Vision Manager
* `Constants.java`
* `RobotContainer.java`

## Auto Routines Study
Until the SuperStructure gets refactored, the following files are useful when studying autonomous routines
* `RobotContainer.java`
* `subsystems/Tracker.java`
* `subsystems/PathHandler.java`
* `subsystems/SuperStructure` (only tangentially as the SuperStructure is deprecated for removal in this particular branch)

All other files are largely unused (some are even blank or fully commented out).