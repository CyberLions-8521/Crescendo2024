// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveModuleConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

//import frc.robot.Util.PPSwerveControllerCommand;

import java.util.HashMap;
import java.util.function.Supplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Util.SwerveModule;
import frc.robot.commands.DriveTele;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Drive() {
    m_gyro.reset();
    //m_gyro.calibrate();

    SmartDashboard.putNumber("Turn P", 0);
    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive FF", 0);
  }
    //odometry
    //gyroscope
    //swerve modules
    //setpositions
    //set states

    private SwerveModule m_bottomRight = new SwerveModule(BOTTOM_RIGHT_DRIVE_PORT, BOTTOM_RIGHT_TURN_PORT, BOTTOM_RIGHT_ENCODER_PORT, BOTTOM_RIGHT_ENCODER_PORT, false);
    private SwerveModule m_bottomLeft = new SwerveModule(BOTTOM_LEFT_DRIVE_PORT, BOTTOM_LEFT_TURN_PORT, BOTTOM_LEFT_ENCODER_PORT, BOTTOM_LEFT_ENCODER_PORT, false);
    private SwerveModule m_topRight = new SwerveModule(TOP_RIGHT_DRIVE_PORT, TOP_RIGHT_TURN_PORT, TOP_RIGHT_ENCODER_PORT, TOP_RIGHT_ENCODER_PORT, false);
    private SwerveModule m_topLeft = new SwerveModule(TOP_LEFT_DRIVE_PORT, TOP_LEFT_TURN_PORT, TOP_LEFT_ENCODER_PORT, TOP_LEFT_ENCODER_PORT, false);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private static Drive m_instance = new Drive();

    public static Drive getInstance(){
      return m_instance;
    }    

    public void setModuleStates(SwerveModuleState[] states){
      SmartDashboard.putNumber("front left desired velocity", states[1].speedMetersPerSecond);
      SmartDashboard.putNumber("front left desired angle", states[1].angle.getDegrees());

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

  public void resetHeading(){
    m_gyro.reset();
  }

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DRIVE_KINEMATICS, this.getDriveHeading(), this.getModulePositions());

  public Pose2d getPose2D(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(getDriveHeading(), getModulePositions(), pose);
  }
  
  public void driveFromChassis(ChassisSpeeds speeds){
    SwerveModuleState[] states = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
    setModuleStates(states);
  }

  public void logData(){
    SmartDashboard.putNumber("Drive Velocity", m_topRight.getDriveVelocity());
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

  public void autonomousRoutine() {
    AutoBuilder.configureHolonomic(
                this::getPose2D, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveFromChassis, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
