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

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public Drive() {
    m_gyro.reset();
    // m_gyro.calibrate();

    SmartDashboard.putNumber("Turn P", TURN_KP);
    SmartDashboard.putNumber("Drive P", DRIVE_KP);
    SmartDashboard.putNumber("Drive D", DRIVE_KD);
    SmartDashboard.putNumber("Drive FF", DRIVE_KFF);
    // publisher = NetworkTableInstance.getDefault()
    //   .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  }
    // private StructArrayPublisher<SwerveModuleState> publisher;

  
    //odometry
    //gyroscope
    //swerve modules
    //setpositions
    //set states

    private static Drive m_instance = new Drive();

  

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

  // public void driveRel(double vx, double vy, double rot ){
  //   var states = Constants.SwerveModuleConstants.kDriveKinematics.toSwerveModuleStates(
  //     ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, Rotation2d.fromDegrees(m_gyro.getYaw()))
  //   );

  //   SwerveDriveKinematics.desaturateWheelSpeeds(states, 4);
  //   m_topLeft.setState(states[0]);
  //   m_topRight.setState(states[1]);
  //   m_bottomLeft.setState(states[2]);
  //   m_bottomRight.setState(states[3]);
  // }


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
    m_gyro.reset();
  }

  public void resetSwerveHeading(){
    SwerveModuleState[] states = getModuleStates();
    kDriveKinematics.resetHeadings(states[0].angle, states[1].angle, states[2].angle, states[3].angle);

  }

  public void logData(){
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

    SmartDashboard.putNumber("Velocity X", getRelativeSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Velocity Y", getRelativeSpeeds().vyMetersPerSecond);

    SmartDashboard.putNumber("top left can coder", m_topLeft.getAbsoluteTurnAngle().getRotations());
    SmartDashboard.putNumber("top right can coder", m_topRight.getAbsoluteTurnAngle().getRotations());
    SmartDashboard.putNumber("bottom left can coder", m_bottomLeft.getAbsoluteTurnAngle().getRotations());
    SmartDashboard.putNumber("bottom right can coder", m_bottomRight.getAbsoluteTurnAngle().getRotations());

    SmartDashboard.putNumber("neo encoder top left", m_topLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("neo encoder top right", m_topRight.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("neo encoder bottom left", m_bottomLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("neo encoder bottom right", m_bottomRight.getTurnAngle().getDegrees());

    SmartDashboard.putNumber("front left actual angle", m_topLeft.getTurnAngle().getDegrees());


  }

  

  @Override
  public void periodic() {
    // publisher.set(new SwerveModuleState[] {
    //   m_topRight.getState(),
    //   m_topLeft.getState(),
    //   m_bottomLeft.getState(),
    //   m_bottomRight.getState()
    // });
    // This method will be called once per scheduler run
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
