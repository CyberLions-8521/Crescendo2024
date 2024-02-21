// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTele;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.Constants.SwerveModuleConstants.*;
public class RobotContainer {
  private final Drive m_drive = Drive.getInstance();
  private final Tracker m_tracker = Tracker.getInstance();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private SendableChooser<Command> autoChooser;// = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    m_drive.setDefaultCommand(new DriveTele(
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX(),
      () -> m_driverController.getRightX()
    , m_drive));
    m_drive.autonomousRoutine();

    // Autos
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    autoChooser.addOption("HELLO JAKCSON", new PathPlannerAuto("Sideways Auto"));
  }

  private void configureBindings() {
    m_driverController.b().onTrue(new InstantCommand(m_drive::readConfigGains));
    m_driverController.button(5).onTrue(new InstantCommand(m_tracker::resetHeading));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return autoChooser.getSelected();
    // return m_drive.followPathCommand("Test Path");
    // return new PathPlannerAuto("Sideways Auto");
  }
}
