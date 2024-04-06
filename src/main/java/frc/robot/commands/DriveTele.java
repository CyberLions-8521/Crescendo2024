// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class DriveTele extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private DoubleSupplier fwd, str, rot;
  private Drive m_drive;
  private final double kDeadZone = 0.15;

  private double modifyInputs(double value, boolean isRot){
    if (Math.abs(value) < kDeadZone) {
      return 0;
    }
    if (isRot) {
      return value * MAX_ANGULAR_VELOCITY;
    }
    return value * MAX_TANGENTIAL_VELOCITY;
  }


   public DriveTele(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, Drive drive) {
    this.m_drive = drive;
    this.fwd = fwd;
    this.str = str;
    this.rot = rot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = -modifyInputs(fwd.getAsDouble(), false);
    double vy = -modifyInputs(str.getAsDouble(), false);
    double omega = -modifyInputs(rot.getAsDouble(), true);

    
    //makes everything like a 3rd person robot
    //might have to add - in front 
    SmartDashboard.putNumber("Angle desired", omega);
    //m_drive.m_gyro.getYaw();
    m_drive.driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega,  Rotation2d.fromDegrees(m_drive.getHeading())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //?
     m_drive.driveFromChassis(new ChassisSpeeds());
  }


}
