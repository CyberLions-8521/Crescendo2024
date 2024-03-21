// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpShoot;
// import frc.robot.commands.Autos;
import frc.robot.commands.DriveTele;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.Source;
import frc.robot.commands.SpeakerShoot;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.Joint.JointState;
import frc.robot.subsystems.HoodWrist;
import frc.robot.subsystems.Joint;
//import frc.robot.subsystems.PathHandler;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Toaster;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.Toaster.ToasterState;
import frc.robot.Constants.SwerveModuleConstants.*;

public class RobotContainer {
  //SUBSYSTEMS
  private final Drive m_drive = Drive.getInstance();
  private final Tracker m_tracker = Tracker.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Hood m_hood = Hood.getInstance();
  private final Toaster m_toaster = Toaster.getInstance(); 
  private final Joint m_joint = Joint.getInstance();
  private final HoodWrist m_hoodWrist = HoodWrist.getInstance();
 

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SuperStructure m_superStructure = SuperStructure.getInstance();
// private final PathHandler m_pathHandler = new PathHandler(m_superStructure);
  public RobotContainer() {
    configureBindings();

    m_drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, m_drive));
  }

  private void configureBindings() {
    //DRIVEBASE
      //m_driverController.button(1).onTrue(new InstantCommand(m_drive::readConfigGains));
      //m_driverController.button(2).onTrue(new InstantCommand(m_drive::rezeroTurnMotors));
      //m_driverController.button(3).onTrue(new InstantCommand(m_drive::resetHeading));
     
    //SUBSYSTEMS
      //Y
      ///m_driverController.button(4).whileTrue(new AmpShoot(m_superStructure));
      //X
      //m_driverController.button(2).whileTrue(new GroundIntake(m_superStructure));
      //B
      // m_driverController.button(3).whileTrue(new SpeakerShoot(m_superStructure));
      //A
      //m_driverController.button(1).whileTrue(new Source(m_superStructure));
        
      // //Triangle
     /*  m_driverController.button(4).whileTrue( new RunCommand(() -> m_toaster.setState(ToasterState.INTAKE)).alongWith(new RunCommand(() -> m_hood.setSpeed(0.8))));
      m_driverController.button(4).onFalse(new RunCommand(() -> m_toaster.setState(ToasterState.OFF)));
      //m_driverController.button(6).whileTrue(new RunCommand(() -> m_hood.setSpeed(HoodConstants.HOOD_SPEED)));
      //m_driverController.button(6).onFalse(new RunCommand(() -> m_hood.setSpeed(0)));

      //X
      m_driverController.button(5).whileTrue(new RunCommand(() -> m_toaster.setState(ToasterState.SPEAKER_SHOOT)));
      m_driverController.button(5).onFalse(new RunCommand(() -> m_toaster.setState(ToasterState.OFF)));*/

      //O
      //m_driverController.button(1).whileTrue(new RunCommand(() -> m_toaster.setState(ToasterState.AMP_SHOOT)));
      //m_driverController.button(1).onFalse(new RunCommand(() -> m_toaster.setState(ToasterState.OFF)));

      //Square
      //m_driverController.button(3).whileTrue(new RunCommand(() -> m_hood.setState(HoodState.ON)));
      //m_driverController.button(3).onFalse(new RunCommand(() -> m_hood.setState(HoodState.OFF)));


    //ELEAVATOR
    //m_driverController.button(2).whileTrue(new RunCommand(() -> m_hood.setSpeed(-0.8)));
      //  m_driverController.button(1).whileTrue(new RunCommand(() -> m_hood.setSpeed(0.8)));

    //m_driverController.button(3).whileTrue(new RunCommand(() -> m_hoodWrist.setSpeed(0.2)));//0.15
    //m_driverController.button(9).whileTrue(new RunCommand(() -> m_hoodWrist.setSpeed(-0.2)));//-0.6
 // m_driverController.button(9).whileTrue(new RunCommand(() -> m_elevator.set(-0.3)));
   // m_driverController.button(10).whileTrue(new RunCommand(() -> m_elevator.set(0.3)));

    //m_driverController.button(4).whileTrue(new RunCommand(() -> m_elevator.setSetpoint(10)));

    // m_driverController.button(2).whileTrue(new RunCommand(() -> m_elevator.setState(ElevatorState.ZERO)));

    // m_driverController.button(1).onTrue(new InstantCommand(m_elevator::configElevatorPID));

    // m_driverController.button(3).onTrue(new InstantCommand(m_elevator::resetEncoder));

    //JOINT
    m_driverController.button(7).whileTrue(new RunCommand(() -> m_joint.set(0.2)));
    m_driverController.button(7).onFalse(new RunCommand(() -> m_joint.setState(JointState.OFF)));
    m_driverController.button(8).whileTrue(new RunCommand(() -> m_joint.set(-0.2)));
    m_driverController.button(8).onFalse(new RunCommand(() -> m_joint.setState(JointState.OFF)));


    //m_driverController.button(1).whileTrue(new RunCommand(() -> m_joint.setSetpoint(Rotation2d.fromRotations(9.3))));
    //m_driverController.button(2).onTrue(new InstantCommand(() -> m_joint.configJointPID()));
    //m_driverController.button(3).onTrue(new RunCommand(() -> m_joint.zero()));
   
    //m_driverController.button(1).whileTrue(new RunCommand(() -> m_joint.setGoal(7, 0)));
    m_driverController.button(2).onTrue(new InstantCommand(() -> m_joint.configJointPID()));
    //0.17
    //0.025
    m_driverController.button(3).onTrue(new InstantCommand(() -> m_joint.rezero()));
    m_driverController.button(4).whileTrue(new RunCommand(() -> m_joint.goToSetpoint()));

    //m_driverController.button(7).whileTrue(new RunCommand(() -> m_joint.set(0.2)));
    //m_driverController.button(8).whileTrue(new RunCommand(() -> m_joint.set(-0.2)));

    //HOOD WRIST
    // m_driverController.button(9).whileTrue(new RunCommand(() -> m_hoodWrist.setSetpoint(4)));
    // m_driverController.button(10).whileTrue(new RunCommand(() -> m_hoodWrist.setSetpoint(0)));

    // m_driverController.button(9).whileTrue(new RunCommand(() -> m_hoodWrist.setSpeed(0.15)));
    // m_driverController.button(10).whileTrue(new RunCommand(() -> m_hoodWrist.setSpeed(-0.15)));

  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
