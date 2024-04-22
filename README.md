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

## Robot Documentation
Images that provide extra information on the robot, its subsystems, as well as videos that showcase the robot responding to the controllers can be found in the [documentation-imgs-vids](https://github.com/CyberLions-8521/Crescendo2024/tree/main/documentation-imgs-vids) directory.

## Robot Subsystems
The subsystems of the robot include the following
* Drivebase - located in `Drive.java`
* Arm - located in `Joint.java`
* Elevator - located in `Elevator.java`.  This subsystem is lifted by the arm, and can be extended in order to reach the SOURCE or score in the AMP.
* Intake/Shooter - located in `Toaster.java`.  This subsystem is within the elevator, and is what is extended when the elevator extends, and as such, is also lifted by the arm.  This is where the NOTE (2024 game piece) sits after it has been taken in by the robot.
* Intake Guides - located in `Hood.java`.  This subsystem consists of plastic rollers that are mounted above the opening of the intake/shooter subsystem.  These rollers are meant to guide the NOTE into the intake to increase the intake tolerance.
* Intake Guide Joint - located in `HoodWrist.java`.  This subsystem controls the motors that act as a "joint" for the "Hood" subsystem, so that it can flip into place when intake, and flip away from the opening when shooting into the SPEAKER..

### Drivebase
The drivebase used a swerve drive with four [MK4i Swerve modules](https://swervedrivespecialties.com/products/mk4i-swerve-module).  The drive motors used were West Coast Product's [Kraken X60, powered by TalonFX](https://wcproducts.com/products/kraken), and the turn motors used were [REV NEO Brushless Motors V1.1](https://www.revrobotics.com/rev-21-1650/).

The built-in Kraken encoder was used as the driving encoder, and a combination of [CTRE's Cancoder (absolute)](https://store.ctr-electronics.com/cancoder/) as well as the built-in NEO (relative) encoder was used for the turning encoder.  The Cancoder was used in order to keep track of absolute azimuthal positioning (as well as to make it so that the wheels would not have to be manually reset to the "zero" position between robot power cycles), and the Cancoder positional data was fed to the NEO relative encoder to run PID and calculate setpoints.

The [IMU (Inertial Measurement Unit)](https://docs.wpilib.org/en/stable/docs/hardware/sensors/accelerometers-hardware.html) used was a [navX2-MXP from Kauai Labs](https://pdocs.kauailabs.com/navx-mxp/).

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
