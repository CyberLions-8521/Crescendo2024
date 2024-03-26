// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveTele;
import frc.robot.commands.ElevatorGoToSetpoint;
import frc.robot.commands.HoodWristGoToSetpoint;
import frc.robot.commands.JointGoToSetpoint;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.HoodWrist.HoodWristState;
import frc.robot.subsystems.Joint.JointState;
import frc.robot.subsystems.HoodWrist;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.PathHandler;
import frc.robot.subsystems.Toaster;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.Toaster.ToasterState;

public class RobotContainer {
  //SUBSYSTEMS
  private final Drive m_drive = Drive.getInstance();
  private final Tracker m_tracker = Tracker.getInstance();
  private final PathHandler m_PathHandler = PathHandler.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Hood m_hood = Hood.getInstance();
  private final Toaster m_toaster = Toaster.getInstance(); 
  private final Joint m_joint = Joint.getInstance();
  private final HoodWrist m_hoodWrist = HoodWrist.getInstance();

  private SendableChooser<Command> m_chooser = new SendableChooser<>(); 


  private Command m_goSource = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 12.4), 
    new JointGoToSetpoint(24.8,0, m_joint), 
    new HoodWristGoToSetpoint(m_hoodWrist, 12.2)
    );

  private Command m_zero = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 0), 
    new JointGoToSetpoint(0,0, m_joint), 
    new HoodWristGoToSetpoint(m_hoodWrist, 0)
  );

  private Command m_shootAmp = Commands.sequence(
    new WaitCommand(2),
    new HoodWristGoToSetpoint(m_hoodWrist, 7.6)
  );

  private Command m_ampScore = Commands.parallel(
    new ElevatorGoToSetpoint(m_elevator, 21), 
    new JointGoToSetpoint(22,0, m_joint),
    m_shootAmp
  );

  private Command m_goSpeaker = Commands.parallel(
    new JointGoToSetpoint(15, 0, m_joint)
  );

  private Command shootCommand = Commands.sequence(
    m_goSpeaker,
    new InstantCommand(() -> m_toaster.setState(ToasterState.SPEAKER_SHOOT)),
    new WaitCommand(3),
    new InstantCommand(() -> m_toaster.setState(ToasterState.OFF))
  );

  private Command intake = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.INTAKE)),
    new RunCommand(() -> m_hood.setSpeed(0.8))
  );

  private Command noIntake = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.OFF)),
    new RunCommand(() -> m_hood.setSpeed(0))
  );

  private Command ampShoot = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.AMP_SHOOT))
  );

  private Command turnOffToaster = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.OFF))
  );

  private Command speakerShoot = Commands.parallel(
    new RunCommand(() -> m_toaster.setState(ToasterState.AMP_SHOOT))
  );

 

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_auxController = new CommandXboxController(2);

  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("shoot", shootCommand);

    m_chooser.addOption("Bottom Taxi", new PathPlannerAuto("Bottom Taxi"));

    SmartDashboard.putData("shoot",shootCommand);
    SmartDashboard.putData("auto",new PathPlannerAuto("Bottom Taxi"));

    SmartDashboard.putData("Chooser", m_chooser);

    m_drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, m_drive));
  }

  private void configureBindings() {
    //DRIVEBASE
      //m_driverController.button(1).onTrue(new InstantCommand(m_drive::rezeroTurnMotors));
      //m_driverController.button(2).onTrue(new InstantCommand(m_drive::resetHeading));
     
    //SUBSYSTEMS
      m_auxController.button(5).whileTrue(intake);
      m_auxController.button(5).onFalse(noIntake);

      m_auxController.button(6).whileTrue(ampShoot);
      m_auxController.button(6).onFalse(turnOffToaster);

      m_auxController.button(7).whileTrue(speakerShoot);
      m_auxController.button(7).onFalse(turnOffToaster);


  m_driverController.button(1).whileTrue(new RunCommand(() -> m_hoodWrist.setJogValue(0.2)));
  m_driverController.button(1).onFalse(new InstantCommand(() -> m_hoodWrist.setState(HoodWristState.OFF)));
  m_driverController.button(2).whileTrue(new RunCommand(() -> m_hoodWrist.setJogValue(-0.2)));
  m_driverController.button(2).onFalse(new InstantCommand(() -> m_hoodWrist.setState(HoodWristState.OFF)));


  
 
//  m_driverController.button(3).whileTrue(new RunCommand(() -> m_elevator.setJog(-0.15)));
//   m_driverController.button(3).onFalse(new RunCommand(() -> m_elevator.setState(ElevatorState.OFF)));
//   m_driverController.button(4).whileTrue(new RunCommand(() -> m_elevator.setJog(0.15)));
//   m_driverController.button(4).onFalse(new RunCommand(() -> m_elevator.setState(ElevatorState.OFF)));

    
    m_driverController.button(7).whileTrue(new RunCommand(() -> m_joint.setJog(0.15)));
    m_driverController.button(7).onFalse(new RunCommand(() -> m_joint.setState(JointState.OFF)));
    m_driverController.button(8).whileTrue(new RunCommand(() -> m_joint.setJog(-0.15)));
    m_driverController.button(8).onFalse(new RunCommand(() -> m_joint.setState(JointState.OFF)));

    
    SmartDashboard.putData("Reset Elevator", new InstantCommand(() -> m_elevator.resetEncoder()));
    SmartDashboard.putData("Reset Joint", new InstantCommand(() -> m_joint.rezero()));
    SmartDashboard.putData("Reset Hood Wrist", new InstantCommand(() -> m_hoodWrist.reZero()));

    m_auxController.button(1).onTrue(m_goSource);
    m_auxController.button(2).onTrue(m_zero);
    m_auxController.button(3).onTrue(m_ampScore);
  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
