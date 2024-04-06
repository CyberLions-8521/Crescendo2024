// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HoodWristConstants;
import frc.robot.Constants.JointConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorGoToSetpoint;
import frc.robot.commands.HoodWristGoToSetpoint;
import frc.robot.commands.JointGoToSetpoint;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.HoodWrist;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.PathHandler;
import frc.robot.subsystems.Toaster;
import frc.robot.subsystems.Tracker;
import frc.robot.Util.mathProfiles;

// Uncomment to play with mini-superstructure
// import frc.robot.subsystems.ElevatedHoodedJoint;

public class RobotContainer {
  //SUBSYSTEMS
  private final Drive m_drive = new Drive();
  private final Elevator m_elevator = new Elevator();
  private final Hood m_hood = new Hood();
  private final HoodWrist m_hoodWrist = new HoodWrist();
  private final Joint m_joint = new Joint();
  private final Toaster m_toaster = new Toaster();

  // Instantiated because they are configured in their constructor.
  // Actual implementation given by PathPlanner calls for configuration within Drive subsystem.
  // This will be done in future codebases.
  private final Tracker m_tracker = new Tracker(m_drive);
  private final PathHandler m_PathHandler = new PathHandler(m_drive, m_tracker/*, m_superStructure*/);

  private SendableChooser<Command> m_chooser = new SendableChooser<>(); 

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_auxController = new CommandXboxController(OperatorConstants.kPartnerControllerPort);
  
  private Command noIntake = Commands.parallel(
    m_toaster.ToasterOffCmd(),
    m_hood.HoodSetSpeedCmd(0)
  );

  private Command m_goSource = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, ElevatorConstants.kSourceSetpoint), 
    new JointGoToSetpoint(JointConstants.kSourceSetpoint,0, m_joint), 
    Commands.sequence(
      Commands.waitSeconds(0.5),
      new HoodWristGoToSetpoint(m_hoodWrist, HoodWristConstants.kSourceSetpoint),
      Commands.parallel(
        m_toaster.ToasterIntakeCmd(),
        m_hood.HoodSetSpeedCmd(HoodConstants.kIntakeSpeed)
      )
    )
  );

  /**
   * In order to score into the amp, we perform the following commands
   * (1) Lift the arm into position, and extend the elevator to the proper length
   * (2) Wait 0.4 seconds before flipping the rollers into position
   */
  private Command m_goAmp = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, ElevatorConstants.kAmpSetpoint), 
    new JointGoToSetpoint(JointConstants.kAmpSetpoint,0, m_joint),
    Commands.sequence(
      Commands.waitSeconds(0.4),
      new HoodWristGoToSetpoint(m_hoodWrist, HoodWristConstants.kAmpSetpoint)
    )
  );

  //ZERO
  private Command m_zero = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 0), 
    new JointGoToSetpoint(0,0, m_joint), 
    new HoodWristGoToSetpoint(m_hoodWrist, 0),
    noIntake
  );

  private Command m_goSideSpeaker = Commands.parallel(
    new JointGoToSetpoint(32, 0, m_joint),
    new HoodWristGoToSetpoint(m_hoodWrist, 0)
  );

  private Command m_goMiddleSpeaker = Commands.parallel(
    new JointGoToSetpoint(29, 0, m_joint),
    new HoodWristGoToSetpoint(m_hoodWrist, 0)
  );

  // The command bound to Button Y of partner controller
  private Command m_goMidSpeakerCommand = Commands.parallel(
    new JointGoToSetpoint(29, 0, m_joint),
    new HoodWristGoToSetpoint(m_hoodWrist, 0)
  );

  // Just trying stuff for fun; largely ignore
  // private Command m_goMiddleSpeakerAuto = ElevatedHoodedJoint.goMidSpeakerCommand(m_elevator, m_hoodWrist, m_joint);
  // private Command m_goMiddleSpeakerTeleop = ElevatedHoodedJoint.goMidSpeakerCommand(m_elevator, m_hoodWrist, m_joint);
  private Command autoMiddleSpeakerShootCommand = m_goMiddleSpeaker.andThen(m_toaster.ToasterSpeakerShootCmd()).withTimeout(3);
  private Command autoSideSpeakerShootCommand = m_goSideSpeaker.andThen(m_toaster.ToasterSpeakerShootCmd()).withTimeout(3);

  public void logData(){
    SmartDashboard.putData("Reset Elevator", new InstantCommand(m_elevator::resetEncoder, m_elevator));
    SmartDashboard.putData("Reset Joint", new InstantCommand(m_joint::rezero, m_joint));
    SmartDashboard.putData("Reset Hood Wrist", new InstantCommand(m_hoodWrist::reZero, m_hoodWrist));
  }
 
  //////////
  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("shoot (Middle)", autoMiddleSpeakerShootCommand);
    NamedCommands.registerCommand("shoot (Side)", autoSideSpeakerShootCommand);
    NamedCommands.registerCommand("Reset Gyro", new InstantCommand(m_drive::resetHeading, m_drive));
    NamedCommands.registerCommand("Rezero Turn Motor", new InstantCommand(m_drive::rezeroTurnMotors, m_drive));
    NamedCommands.registerCommand("down", m_zero);

    m_chooser.addOption("Top Taxi", new PathPlannerAuto("Top Taxi"));
    m_chooser.addOption("Bottom Taxi", new PathPlannerAuto("Bottom Taxi"));
    m_chooser.addOption("Middle Taxi", new PathPlannerAuto("Middle Taxi"));
    m_chooser.addOption("Practice", new PathPlannerAuto("HOLA"));
    m_chooser.addOption("no auto", Commands.none());

    SmartDashboard.putData("shoot (Middle)",autoMiddleSpeakerShootCommand);
    SmartDashboard.putData("shoot (Side)",autoSideSpeakerShootCommand);
    SmartDashboard.putData("Chooser", m_chooser);

    m_goSource.setName("GoToSource");
    m_zero.setName("Zero");
    m_goAmp.setName("GoAmp");

    // Note that MathUtil.applyDeadband() will automatically scale its return value between -1 and 1
    // Note also that for field oriented driving, the +x direction relative to the field is +y relative to the driver
    // Similarly, the +y direction relative to the field is the -x direction relative to the driver
    // Hence, we pass leftY, leftX in that order to drive, when drive asks for xSpeed, ySpeed (in that order)
    var m_driveCommand = new RunCommand(
          () -> 
          m_drive.drive(
            -mathProfiles.exponentialDrive(MathUtil.applyDeadband(m_driverController.getLeftY(), DriveConstants.kDriveDeadband), 2),
            -mathProfiles.exponentialDrive(MathUtil.applyDeadband(m_driverController.getLeftX(), DriveConstants.kDriveDeadband), 2),
            -MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.kDriveDeadband),
            true,
            false),
          m_drive);
    m_driveCommand.setName("DriveCommand");

    m_drive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      m_driveCommand
    );

    m_toaster.setDefaultCommand(m_toaster.ToasterOffCmd());
    m_hood.setDefaultCommand(m_hood.HoodSetSpeedCmd(0));
    m_hoodWrist.setDefaultCommand(new InstantCommand(() -> m_hoodWrist.setSpeed(0), m_hoodWrist));
    m_joint.setDefaultCommand(m_joint.JointSetOffCmd());
}

  private void configureBindings() {
    logData();
    //DRIVEBASE
    m_driverController.a().onTrue(new InstantCommand(m_drive::rezeroTurnMotors, m_drive));
    m_driverController.b().onTrue(new InstantCommand(m_drive::resetHeading, m_drive));

    // Master controller
    m_driverController.back().whileTrue(m_joint.JointSetJogCmd(0.15));
    m_driverController.start().whileTrue(m_joint.JointSetJogCmd(-0.15));
    
    // Partner controller
    m_auxController.leftBumper().whileTrue(m_toaster.ToasterAmpShootCmd());
    m_auxController.rightBumper().whileTrue(m_toaster.ToasterSpeakerShootCmd());
    m_auxController.back().whileTrue(
      m_toaster.ToasterIntakeCmd()
      .alongWith(m_hood.HoodSetSpeedCmd(-0.8))
    );
    m_auxController.start().whileTrue(m_hood.HoodSetSpeedCmd(0.8).repeatedly());
    m_auxController.a().onTrue(m_goSource);
    m_auxController.b().onTrue(m_zero);
    m_auxController.x().onTrue(m_goAmp);
    m_auxController.y().onTrue(m_goMidSpeakerCommand);
  }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
