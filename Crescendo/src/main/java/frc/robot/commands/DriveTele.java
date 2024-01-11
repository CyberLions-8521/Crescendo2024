// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY;
import static frc.robot.Constants.SwerveModuleConstants.DRIVE_KINEMATICS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class DriveTele extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Drive m_drive = Drive.getInstance();

  private DoubleSupplier fwd, str, rot;
  private Drive drive;

  private double modifyInputs(double value, boolean isRot){
    if(isRot){
      //deadzone
      if(Math.abs(value) < 0.15){
        value = 0;
      
      }
      return value * MAX_ANGULAR_VELOCITY;
    }

      else{
        if(Math.abs(value) < 0.15){
          value = 0;
        }
        return value * MAX_TANGENTIAL_VELOCITY;
      }
  }

  public void driveFromChassis(ChassisSpeeds speeds){
    SwerveModuleState[] states = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY);
    m_drive.setModuleStates(states);
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
    double vx = modifyInputs(fwd.getAsDouble(), false);
    double vy = modifyInputs(str.getAsDouble(), false);
    double omega = modifyInputs(rot.getAsDouble(), true);

    //makes everything like a 3rd person robot
    //might have to add - in front 
    driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, m_drive.getDriveHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //?
     driveFromChassis(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
