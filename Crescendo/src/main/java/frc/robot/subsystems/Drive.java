// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Util.SwerveModule;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Drive() {
    m_gyro.reset();
    m_gyro.calibrate();

    SmartDashboard.putNumber("Turn P", 0);
    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive FF", 0);
  }
    //odometry
    //gyroscope
    //swerve modules
    //setpositions
    //set states

    private SwerveModule m_bottomRight = new SwerveModule(SwerveModuleConstants.BOTTOM_RIGHT_DRIVE_PORT, SwerveModuleConstants.BOTTOM_RIGHT_TURN_PORT, SwerveModuleConstants.BOTTOM_RIGHT_ENCODER_PORT, SwerveModuleConstants.BOTTOM_RIGHT_ENCODER_PORT, false);
    private SwerveModule m_bottomLeft = new SwerveModule(SwerveModuleConstants.BOTTOM_LEFT_DRIVE_PORT, SwerveModuleConstants.BOTTOM_LEFT_TURN_PORT, SwerveModuleConstants.BOTTOM_LEFT_ENCODER_PORT, SwerveModuleConstants.BOTTOM_LEFT_ENCODER_PORT, false);
    private SwerveModule m_topRight = new SwerveModule(SwerveModuleConstants.TOP_RIGHT_DRIVE_PORT, SwerveModuleConstants.TOP_RIGHT_TURN_PORT, SwerveModuleConstants.TOP_RIGHT_ENCODER_PORT, SwerveModuleConstants.TOP_RIGHT_ENCODER_PORT, false);
    private SwerveModule m_topLeft = new SwerveModule(SwerveModuleConstants.TOP_LEFT_DRIVE_PORT, SwerveModuleConstants.TOP_LEFT_TURN_PORT, SwerveModuleConstants.TOP_LEFT_ENCODER_PORT, SwerveModuleConstants.TOP_LEFT_ENCODER_PORT, false);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private Drive m_instance = new Drive();

    public Drive getInstance(){
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
    SwerveModulePosition[] modulePositions = {m_topLeft.getModulePosition(), m_topRight.getModulePosition(), m_bottomLeft.getModulePosition(), rearRight.getModulePosition()};
    return modulePositions;
  }

  public Rotation2d getDriveHeading(){
    return m_gyro.getRotation2d();
  }

  public void resetHeading(){
    m_gyro.reset();
  }


    // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveModuleConstants.DRIVE_KINEMATICS, m_gyro.getAngle(), getSwerveModulePositions());

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

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
