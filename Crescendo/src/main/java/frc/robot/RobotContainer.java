// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.PathHandler;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.Constants.SwerveModuleConstants.*;

public class RobotContainer {
  private final Drive m_drive = Drive.getInstance();
  private final Tracker m_tracker = Tracker.getInstance();
  private final PathHandler m_pathHandler = PathHandler.getInstance();
  private final SuperStructure m_superStructure = SuperStructure.getInstance();

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
    //return Autos.exampleAuto(m_exampleSubsystem);
    return m_pathHandler.followPath("First Path");
  }
}
