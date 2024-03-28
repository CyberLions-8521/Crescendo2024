// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveModuleConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Constants;

//import frc.robot.Util.PPSwerveControllerCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.Util.SwerveModule;
import frc.robot.Util.SwerveUtils;

public class Drive extends SubsystemBase {
  public Drive() {
    m_gyro.reset();

    SmartDashboard.putNumber("Turn P", TURN_KP);
    SmartDashboard.putNumber("Drive P", DRIVE_KP);
    SmartDashboard.putNumber("Drive D", DRIVE_KD);
    SmartDashboard.putNumber("Drive FF", DRIVE_KFF);
  }

  private static Drive m_instance = new Drive();

  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(1.8);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public static Drive getInstance(){
    return m_instance;
  }
  
  private SwerveModule m_bottomRight = new SwerveModule(BOTTOM_RIGHT_DRIVE_PORT, BOTTOM_RIGHT_TURN_PORT, BOTTOM_RIGHT_ENCODER_PORT, BOTTOM_RIGHT_ENCODER_OFFSET, true);
  private SwerveModule m_bottomLeft = new SwerveModule(BOTTOM_LEFT_DRIVE_PORT, BOTTOM_LEFT_TURN_PORT, BOTTOM_LEFT_ENCODER_PORT, BOTTOM_LEFT_ENCODER_OFFSET, true);
  private SwerveModule m_topRight = new SwerveModule(TOP_RIGHT_DRIVE_PORT, TOP_RIGHT_TURN_PORT, TOP_RIGHT_ENCODER_PORT, TOP_RIGHT_ENCODER_OFFSET, true);
  private SwerveModule m_topLeft = new SwerveModule(TOP_LEFT_DRIVE_PORT, TOP_LEFT_TURN_PORT, TOP_LEFT_ENCODER_PORT, TOP_LEFT_ENCODER_OFFSET, true);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public void setModuleStates(SwerveModuleState[] states){
    SmartDashboard.putNumber("front left desired velocity", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("front left desired angle", states[0].angle.getDegrees());

    m_topLeft.setState(states[0]);
    m_topRight.setState(states[1]);
    m_bottomLeft.setState(states[2]);
    m_bottomRight.setState(states[3]);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_TANGENTIAL_VELOCITY;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_TANGENTIAL_VELOCITY;
    double rotDelivered = m_currentRotation * DriveConstants.MAX_ANGULAR_VELOCITY;
    
    var swerveModuleStates =
    Constants.SwerveModuleConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getHeading()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            );
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_TANGENTIAL_VELOCITY);
    m_topLeft.setState(swerveModuleStates[0]);
    m_topRight.setState(swerveModuleStates[1]);
    m_bottomLeft.setState(swerveModuleStates[2]);
    m_bottomRight.setState(swerveModuleStates[3]);
  }

   public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {m_topLeft.getModulePosition(), m_topRight.getModulePosition(), m_bottomLeft.getModulePosition(), m_bottomRight.getModulePosition()};
    return modulePositions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {m_topLeft.getState(), m_topRight.getState(), m_bottomLeft.getState(), m_bottomRight.getState()};
    return moduleStates;
  }

  public Rotation2d getDriveHeading(){
    return m_gyro.getRotation2d();
  }

  public ChassisSpeeds getRelativeSpeeds(){
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void readConfigGains(){
    m_topLeft.configGains();
    m_topRight.configGains();
    m_bottomLeft.configGains();
    m_bottomRight.configGains();
  }
  
  public void rezeroTurnMotors(){
    m_topLeft.rezeroTurnMotors();
    m_topRight.rezeroTurnMotors();
    m_bottomLeft.rezeroTurnMotors();
    m_bottomRight.rezeroTurnMotors();
  }
  
  public void driveFromChassis(ChassisSpeeds speeds){
    SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
    setModuleStates(states);
  }

  public void resetHeading(){
    m_gyro.zeroYaw();
  }

  public void resetSwerveHeading(){
    SwerveModuleState[] states = getModuleStates();
    kDriveKinematics.resetHeadings(states[0].angle, states[1].angle, states[2].angle, states[3].angle);
  }

  public double getHeading() {
    return -m_gyro.getYaw();
  }

  public void logData(){
    SmartDashboard.putNumber("Turn Angle", m_topRight.getTurnAngle().getDegrees());

    SmartDashboard.putNumber("Absolute Turn", m_topRight.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("Gyro Degrees", m_gyro.getAngle());
    
    SmartDashboard.putNumber("front left abs", m_topLeft.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("front right abs", m_topRight.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear left abs", m_bottomLeft.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear right abs", m_bottomRight.getAbsoluteTurnAngle().getDegrees());

    SmartDashboard.putNumber("front left relative", m_topLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("front right relative", m_topRight.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear left relative", m_bottomLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear right relative", m_bottomRight.getTurnAngle().getDegrees());

    SmartDashboard.putNumber("Velocity X", getRelativeSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Velocity Y", getRelativeSpeeds().vyMetersPerSecond);

    SmartDashboard.putNumber("front left actual angle", m_topLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("front left actual velocity", Math.abs(m_topLeft.getDriveVelocity()));
    
    SmartDashboard.putNumber("Heading", getHeading());

  }

  @Override
  public void periodic() {
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
