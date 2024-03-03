// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.commands.DriveTele;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Toaster;
//import frc.robot.commands.Shoot;
//import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.Constants.SwerveModuleConstants.*;

public class RobotContainer {
  private final Drive m_drive = Drive.getInstance();
  private final Tracker m_tracker = Tracker.getInstance();
  private final Elevator m_elevator = new Elevator();
  private final Toaster m_toaster = new Toaster();
  //rivate final Shoot m_shoot = new Shoot(m_toaster);
  //private final SuperStructure m_superStructure = new SuperStructure();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_drive.setDefaultCommand(new DriveTele(
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX(),
      () -> m_driverController.getRightX()
    , m_drive));
  }

  private void configureBindings() {
    //DRIVEBASE
    //m_driverController.b().onTrue(new InstantCommand(m_drive::readConfigGains));
    //m_driverController.a().onTrue(new InstantCommand(m_drive::resetHeading));
    
    //TOASTER
     m_driverController.button(4).whileTrue(new RunCommand(() -> m_toaster.intake()));
    m_driverController.button(3)
      .whileTrue( (new RunCommand(() -> m_toaster.setShooter(0.8)))
      .alongWith( new WaitCommand(0.75)
      .andThen(new RunCommand(() -> m_toaster.setHolder(0.9))) ));
    
    m_driverController.button(1).whileTrue(new RunCommand(() -> m_toaster.setHolder(0.9)));
    

    //ELEAVATOR
    //y
    m_driverController.button(5).whileTrue(new RunCommand(() -> m_elevator.set(0.3)));
    //x
   m_driverController.button(6).whileTrue(new RunCommand(() -> m_elevator.set(-0.3)));
   m_driverController.button(2).whileTrue(new RunCommand(() -> m_elevator.zero()));
     
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
