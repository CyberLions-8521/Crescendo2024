# README
The main contributors of the original code base included the following senior programmers from the class of 2024
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
The robot was controlled using two XBox/Logitech F310 gamepads.  The master controller had the following actions
* Analog Joysticks - controlled the swerve drivebase
* Button A - re-zeroed the turn motors on the swerve drivebase
* Button B - reset the heading of the robot
* Button Back - manually lifts the arm
* Button Start - manually lowers the arm

The partner controller had the following actions
* Button Left Bumper - turn on motors of shooter to shoot into the AMP
* Button Right Bumper - turn on motors of shooter to shoot into the SPEAKER
* Button Back - turns on motors for intaking; mainly meant for testing purposes
* Button Start - turns on motors for outtaking/shooting; mainly meant for testing purposes
* Button A - raise, extend, and flip robot subsystems into position for the SOURCE and turn on necessary motors to intake NOTES
* Button B - lower, retract, and flip robot subsystems back down to default positions
* Button X - raise, extend, and flip robot subsystems into position for the AMP
* Button Y - raise, extend, and flip robot subsystems into position to score in the SPEAKER

## Unused Files
The following files were largely unused:
* `subsystems/VisionManager.java`
* `LimelightHelpers.java`