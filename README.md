# README
The `post-ocr` branch, if it has not yet been merged with branch `main`, is a current work-in-progress branch being done by CyberLions mentor Kyle Vu.  The main contributors of the original code base included the following senior programmers from the class of 2024
* [Jackson Nguyen](https://github.com/jackhnguyen)
* [Rachel Fernandez](https://github.com/PokaDoka1)
* [Kevin Vo](https://github.com/EV3KevinDEV)

Much of the original code base borrows code from three different sources
* WPILib's Command Based Programming framework
* Team 7157 Mubotic's 2023-2024 code (courtesy of [Aaron Yoon](https://github.com/awesomeyooner))
* Team 2710 Jetstream's 2023-2024 code (courtesy of [Kevin Vo](https://github.com/EV3KevinDEV))

If you are studying the Command Based Programming framework for teleop, then the focus of your study should be on the following files in `src/main/java/frc/robot`.  Note that in any of the subsystem files, the `switch` statements involving subsystem state can be ignored.  The code has been refactored so that it no longer depends on subsystem states.  As of April 2, 2024, the code has yet to be tested on a physical robot.  The code otherwise compiles and builds without error.
* `commands/ElevatorGoToSetpoint.java`
* `commands/HoodWristGoToSetpoint.java`
* `commands/JointGoToSetpoint.java`
* All subsystems within the `subsystems` directory, with the exception of PathHandler, SuperStructure, Tracker, and Vision Manager
* `Constants.java`
* `RobotContainer.java`

Until the SuperStructure gets refactored, the following files are useful when studying autonomous routines
* `RobotContainer.java`
* `subsystems/Tracker.java`
* `subsystems/PathHandler.java`
* `subsystems/SuperStructure` (only tangentially as the SuperStructure is deprecated for removal in this particular branch)

All other files are largely unused (some are even blank or fully commented out).