// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveModuleConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;


//import frc.robot.Util.PPSwerveControllerCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.Util.SwerveModule;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Drive() {
    m_gyro.reset();
    //m_gyro.calibrate();

    SmartDashboard.putNumber("Turn P", TURN_KP);
    SmartDashboard.putNumber("Drive P", DRIVE_KP);
    SmartDashboard.putNumber("Drive D", DRIVE_KD);
    SmartDashboard.putNumber("Drive FF", DRIVE_KFF);
  }
    //odometry
    //gyroscope
    //swerve modules
    //setpositions
    //set states

    private SwerveModule m_bottomRight = new SwerveModule(BOTTOM_RIGHT_DRIVE_PORT, BOTTOM_RIGHT_TURN_PORT, BOTTOM_RIGHT_ENCODER_PORT, BOTTOM_RIGHT_ENCODER_OFFSET, true);
    private SwerveModule m_bottomLeft = new SwerveModule(BOTTOM_LEFT_DRIVE_PORT, BOTTOM_LEFT_TURN_PORT, BOTTOM_LEFT_ENCODER_PORT, BOTTOM_LEFT_ENCODER_OFFSET, true);
    private SwerveModule m_topRight = new SwerveModule(TOP_RIGHT_DRIVE_PORT, TOP_RIGHT_TURN_PORT, TOP_RIGHT_ENCODER_PORT, TOP_RIGHT_ENCODER_OFFSET, true);
    private SwerveModule m_topLeft = new SwerveModule(TOP_LEFT_DRIVE_PORT, TOP_LEFT_TURN_PORT, TOP_LEFT_ENCODER_PORT, TOP_LEFT_ENCODER_OFFSET, true);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private static Drive m_instance = new Drive();

    public static Drive getInstance(){
      return m_instance;
    }    

    public void setModuleStates(SwerveModuleState[] states){
      SmartDashboard.putNumber("front left desired velocity", states[0].speedMetersPerSecond);
      SmartDashboard.putNumber("front left desired angle", states[0].angle.getDegrees());
      SmartDashboard.putNumber("front left error", Math.abs(Math.abs(states[0].speedMetersPerSecond) - Math.abs(m_topLeft.getDriveVelocity())));

      m_topLeft.setState(states[0]);
      m_topRight.setState(states[1]);
      m_bottomLeft.setState(states[2]);
      m_bottomRight.setState(states[3]);
  }

   public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {m_bottomRight.getModulePosition(), m_bottomLeft.getModulePosition(), m_topRight.getModulePosition(), m_topLeft.getModulePosition()};
    return modulePositions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {m_bottomLeft.getState(), m_bottomLeft.getState(), m_bottomLeft.getState(), m_bottomLeft.getState()};
    return moduleStates;
  }

  public Rotation2d getDriveHeading(){
    return m_gyro.getRotation2d();
  }

  public ChassisSpeeds getRelativeSpeeds(){
    return DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }
  
  public void readConfigGains(){
    m_topLeft.configGains();
    m_topRight.configGains();
    m_bottomLeft.configGains();
    m_bottomRight.configGains();

  }
  
  public void driveFromChassis(ChassisSpeeds speeds){
    SwerveModuleState[] states = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
    setModuleStates(states);
  }

  public void logData(){
    SmartDashboard.putNumber("Drive Velocity", Math.abs(m_topRight.getDriveVelocity()));
    SmartDashboard.putNumber("Turn Angle", m_topRight.getTurnAngle().getDegrees());

    SmartDashboard.putNumber("Absolute Turn", m_topRight.getAbsoluteTurnAngle().getDegrees());

    SmartDashboard.putNumber("Gyro Degrees", m_gyro.getAngle());

    SmartDashboard.putNumber("front left abs", m_topLeft.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("front right abs", m_topRight.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear left abs", m_bottomLeft.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear right abs", m_bottomRight.getAbsoluteTurnAngle().getDegrees());

    SmartDashboard.putNumber("front left", m_topLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("front right", m_topRight.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear left", m_bottomLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear right", m_bottomRight.getTurnAngle().getDegrees());
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
