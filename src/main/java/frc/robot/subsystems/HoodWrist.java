// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.HoodWristConstants;

public class HoodWrist extends SubsystemBase {
  
  //CONSTRUCTOR
  public HoodWrist() {
    configMotors();
    resetEncoder();
  }
  
  //MOTOR OBJECT
  private CANSparkMax m_hoodWristMaster = new CANSparkMax(MotorConstants.HOOD_WRIST_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_hoodEncoder = m_hoodWristMaster.getEncoder();
  
  //LIMIT SWITCH
  private DigitalInput m_limitSwitch = new DigitalInput(1);

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(95, 85));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(getPosition(), 0);

  //PID CONTROLLER
  private SparkPIDController m_hoodController = m_hoodWristMaster.getPIDController();

  private RelativeEncoder m_hoodWristEncoder = m_hoodWristMaster.getEncoder();

  private double getPosition() {
    return m_hoodEncoder.getPosition();
  }

  //SET MOTOR OUTPUT METHODS
  public void setSpeed(double value) {
    m_hoodWristMaster.set(value);
  }

  //SETPOINT METHODS
  public void refreshSetpoint() {
    m_setpoint = new TrapezoidProfile.State(getPosition(), m_hoodWristEncoder.getVelocity());
  }

  public void setGoal(double desiredPosition, double desiredVelocity) {
    m_goal = new TrapezoidProfile.State(desiredPosition, desiredVelocity);
    goToSetpoint();
  }

  private void goToSetpoint() {
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
    m_hoodController.setReference(m_setpoint.position, ControlType.kPosition);
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(m_goal.position, getPosition(), 0.1);
  }

  //ZERO METHODS / Encoders
  public void resetEncoder() {
    m_hoodEncoder.setPosition(0);
  }

  public boolean atZero() {
    return m_limitSwitch.get();
  }

  public void zero() {
    //IF LIMIT SWITCH IS NOT CLICKED --> GO BACKWARDS
    if (!atZero()) {
      setSpeed(-0.1);
    } else {
      setSpeed(0);
      resetEncoder();
    }
  }

  public void reZero() {
    m_hoodEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    logData();
  }

  private void logData(){
    SmartDashboard.putNumber("hood wrist Encoder Position", m_hoodEncoder.getPosition());
    SmartDashboard.putNumber("hood wrist at Position", m_setpoint.position);
    SmartDashboard.putNumber("Hood Wrist Setpoint Position", m_setpoint.position);
    SmartDashboard.putNumber("Hood Wrist Goal Position", m_goal.position);
    SmartDashboard.putNumber("hood Wrist KP", m_hoodController.getP());
    SmartDashboard.putNumber("hood Wrist KD", m_hoodController.getD());
  }

  private void configMotors(){
    m_hoodWristMaster.restoreFactoryDefaults();
    m_hoodWristMaster.setInverted(false);
    m_hoodWristMaster.setIdleMode(IdleMode.kBrake);
    m_hoodController.setP(HoodWristConstants.HOOD_WRIST_KP);
    m_hoodController.setD(HoodWristConstants.HOOD_WRIST_KD);
    m_hoodWristMaster.burnFlash(); 
  }
}
