// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HoodWrist;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.HoodWrist.HoodWristState;
import frc.robot.subsystems.Joint.JointState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class HoodWristGoToSetpoint extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HoodWrist m_hoodWrist;
  private final double setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HoodWristGoToSetpoint(HoodWrist hoodWrist, double setpoint) {
    m_hoodWrist = hoodWrist;
    this.setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hoodWrist);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hoodWrist.refreshSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hoodWrist.setGoal(setpoint, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hoodWrist.setState(HoodWristState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hoodWrist.atSetpoint();
  }
}
