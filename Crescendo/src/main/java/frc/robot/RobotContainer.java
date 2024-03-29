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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.mathProfiles;
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
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Toaster.ToasterState;

public class RobotContainer {
  //SUBSYSTEMS

  private final Drive m_drive = Drive.getInstance();
  //private final Tracker m_tracker = Tracker.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Hood m_hood = Hood.getInstance();
  private final Toaster m_toaster = Toaster.getInstance(); 
  private final Joint m_joint = Joint.getInstance();
  private final HoodWrist m_hoodWrist = HoodWrist.getInstance();
  private final PathHandler m_PathHandler = PathHandler.getInstance();

  private SendableChooser<Command> m_chooser = new SendableChooser<>(); 

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_auxController = new CommandXboxController(2);

  //INTAKING
  private Command intake = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.INTAKE)),
    new RunCommand(() -> m_hood.setSpeed(-0.8))
  );

  private Command noIntake = Commands.parallel(
    new InstantCommand(() -> m_toaster.setState(ToasterState.OFF)),
    new InstantCommand(() -> m_hood.setSpeed(0))
  );

  //SOURCE
  private Command m_intakeSource = Commands.sequence(
    new WaitCommand(0.5),
    new HoodWristGoToSetpoint(m_hoodWrist, 10),
    intake
  );

  private Command m_goSource = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 14), 
    new JointGoToSetpoint(24.5,0, m_joint), 
    m_intakeSource
  );

  //AMP SCORING
  private Command m_hoodWristToAmp = Commands.sequence(
    new WaitCommand(.4),
    new HoodWristGoToSetpoint(m_hoodWrist, 6)
  );

    private Command m_ampShoot = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.AMP_SHOOT))
  );

  private Command m_goAmp = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 23), 
    new JointGoToSetpoint(22,0, m_joint),
    m_hoodWristToAmp
  );

  //ZERO
  private Command m_zero = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 0), 
    new JointGoToSetpoint(0,0, m_joint), 
    new HoodWristGoToSetpoint(m_hoodWrist, 0),
    noIntake
  );

  //SPEAKER
  private Command m_speakerShoot = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.SPEAKER_SHOOT))
  );

  private Command m_goSideSpeaker = Commands.parallel(
    new JointGoToSetpoint(32, 0, m_joint),
    new HoodWristGoToSetpoint(m_hoodWrist, 0)
  );

  private Command m_goMiddleSpeaker = Commands.parallel(
    new JointGoToSetpoint(29, 0, m_joint),
    new HoodWristGoToSetpoint(m_hoodWrist, 0)
  );

  // AUTO SPEAKER SHOOTING
  private Command autoMiddleSpeakerShootCommand = Commands.sequence(
    m_goMiddleSpeaker,
    new InstantCommand(() -> m_toaster.setState(ToasterState.SPEAKER_SHOOT)),
    new WaitCommand(3),
    new InstantCommand(() -> m_toaster.setState(ToasterState.OFF))
  );

  private Command autoSideSpeakerShootCommand = Commands.sequence(
    m_goSideSpeaker,
    new InstantCommand(() -> m_toaster.setState(ToasterState.SPEAKER_SHOOT)),
    new WaitCommand(3),
    new InstantCommand(() -> m_toaster.setState(ToasterState.OFF))
  );

  

  //TURN OFF TOASTER
  private Command turnOffToaster = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.OFF))
  );

  public void logData(){
    SmartDashboard.putData("Reset Elevator", new InstantCommand(() -> m_elevator.resetEncoder()));
    SmartDashboard.putData("Reset Joint", new InstantCommand(() -> m_joint.rezero()));
    SmartDashboard.putData("Reset Hood Wrist", new InstantCommand(() -> m_hoodWrist.reZero()));
  }
 
  //////////
  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("shoot (Middle)", autoMiddleSpeakerShootCommand);
    NamedCommands.registerCommand("shoot (Side)", autoSideSpeakerShootCommand);
    NamedCommands.registerCommand("Reset Gyro", new InstantCommand(() -> m_drive.resetHeading()));
    NamedCommands.registerCommand("Rezero Turn Motor", new InstantCommand(() -> m_drive.rezeroTurnMotors()));
    NamedCommands.registerCommand("down", m_zero);

    m_chooser.addOption("Top Taxi", new PathPlannerAuto("Top Taxi"));
    m_chooser.addOption("Bottom Taxi", new PathPlannerAuto("Bottom Taxi"));
    m_chooser.addOption("Middle Taxi", new PathPlannerAuto("Middle Taxi"));
    m_chooser.addOption("Practice", new PathPlannerAuto("HOLA"));

    m_chooser.addOption("no auto", Commands.none());
    SmartDashboard.putData("shoot (Middle)",autoMiddleSpeakerShootCommand);
    SmartDashboard.putData("shoot (Side)",autoSideSpeakerShootCommand);

   // SmartDashboard.putData("auto",new PathPlannerAuto("Bottom Taxi"));
    //SmartDashboard.putData("plz follow path",new PathPlannerAuto("Test Path"));

    SmartDashboard.putData("Chooser", m_chooser);

    //m_drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, m_drive));

    m_drive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> 
          m_drive.drive(
            -mathProfiles.exponentialDrive(MathUtil.applyDeadband(m_driverController.getLeftY(), DriveConstants.kDriveDeadband), 2),
            -mathProfiles.exponentialDrive(MathUtil.applyDeadband(m_driverController.getLeftX(), DriveConstants.kDriveDeadband), 2),
              -MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.kDriveDeadband),
              true, false),
          m_drive));
}

  private void configureBindings() {
    logData();
    //DRIVEBASE
    m_driverController.button(1).onTrue(new InstantCommand(m_drive::rezeroTurnMotors));
    m_driverController.button(2).onTrue(new InstantCommand(m_drive::resetHeading));

    //HOOD WRIST
    // m_driverController.button(3).whileTrue(new RunCommand(() -> m_hoodWrist.setJogValue(0.2)));
    // m_driverController.button(3).onFalse(new InstantCommand(() -> m_hoodWrist.setState(HoodWristState.OFF)));
    // m_driverController.button(4).whileTrue(new RunCommand(() -> m_hoodWrist.setJogValue(-0.2)));
    // m_driverController.button(4).onFalse(new InstantCommand(() -> m_hoodWrist.setState(HoodWristState.OFF)));
  
    //ELEVATOR
    m_driverController.button(7).whileTrue(new RunCommand(() -> m_elevator.setJog(-0.15)));
    m_driverController.button(7).onFalse(new RunCommand(() -> m_elevator.setState(ElevatorState.OFF)));
    m_driverController.button(8).whileTrue(new RunCommand(() -> m_elevator.setJog(0.15)));
    m_driverController.button(8).onFalse(new RunCommand(() -> m_elevator.setState(ElevatorState.OFF)));

    //JOINT
    // m_driverController.button(7).whileTrue(new RunCommand(() -> m_joint.setJog(0.15)));
    // m_driverController.button(7).onFalse(new RunCommand(() -> m_joint.setState(JointState.OFF)));
    // m_driverController.button(8).whileTrue(new RunCommand(() -> m_joint.setJog(-0.15)));
    // m_driverController.button(8).onFalse(new RunCommand(() -> m_joint.setState(JointState.OFF)));
      
    ///////////////////
    m_auxController.button(5).whileTrue(m_ampShoot);
    m_auxController.button(5).onFalse(turnOffToaster);

    m_auxController.button(6).whileTrue(m_speakerShoot);
    m_auxController.button(6).onFalse(turnOffToaster);

    m_auxController.button(7).whileTrue(new RunCommand(() -> m_toaster.setState(ToasterState.INTAKE)).alongWith(
    new RunCommand(() -> m_hood.setSpeed(-0.8))));
    m_auxController.button(7).onFalse(new RunCommand(() -> m_toaster.setState(ToasterState.OFF)).alongWith(
    new RunCommand(() -> m_hood.setSpeed(0))));

    m_auxController.button(8).whileTrue( new RunCommand(() -> m_hood.setSpeed(0.8)));
    m_auxController.button(8).onFalse(new RunCommand(() -> m_hood.setSpeed(0)));

    m_auxController.button(1).onTrue(m_goSource);
    m_auxController.button(2).onTrue(m_zero);
    m_auxController.button(3).onTrue(m_goAmp);
    // m_auxController.button(4).onTrue(m_goSpeaker);
    m_auxController.button(4).onTrue(new ElevatorGoToSetpoint(m_elevator, 10).alongWith(new 
      JointGoToSetpoint(29,0, m_joint)));

  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    //return m_chooser.getSelected();
    //return null;
    //return new PathPlannerAuto("Bottom Taxi");
    return m_chooser.getSelected();
  }
}
