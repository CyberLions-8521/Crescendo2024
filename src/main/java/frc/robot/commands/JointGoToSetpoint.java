// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Joint;
// import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Joint.JointState;
// import frc.robot.subsystems.SuperStructure.SuperStructureState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class JointGoToSetpoint extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Joint m_joint;
  private final double goalPosition;
  private final double goalVelocity;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JointGoToSetpoint(double desiredGoal, double desiredVelocity, Joint joint) {
    m_joint = joint;
    goalPosition = desiredGoal;
    goalVelocity = desiredVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(joint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_joint.refreshSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_joint.setGoal(goalPosition, goalVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joint.setState(JointState.OFF);
    //_joint.refreshSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_joint.atSetpoint());
  }
}
