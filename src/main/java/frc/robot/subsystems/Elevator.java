// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
public class Elevator extends SubsystemBase {

  //CONSTRUCTOR
  public Elevator() {
    configMotors();
    resetEncoder();
    
    SmartDashboard.putNumber("Elevator kP", ElevatorConstants.ELEVATOR_KP);
    SmartDashboard.putNumber("Elevator kD", ElevatorConstants.ELEVATOR_KD);
  }
  //MOTOR OBJECTS
  private CANSparkMax m_elevatorMaster = new CANSparkMax(MotorConstants.ELEVATOR_MASTER_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_elevatorEncoder = m_elevatorMaster.getEncoder();

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(200, 100));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
  
  //PID CONTROLLER OBJECT
  private SparkPIDController m_elevatorController = m_elevatorMaster.getPIDController();

  private double getPosition() {
    return m_elevatorEncoder.getPosition();
  }

  //SET MOTOR OUTPUT METHODS
  private void set(double value) {
    m_elevatorMaster.set(value);
  }

  public void setWithTuneValue() {
    set(getPosition() * ElevatorConstants.kAntiGravityMultiplier);
  }

  //SETPOINT METHODS
  public void refreshSetpoint() {
    m_setpoint = new TrapezoidProfile.State(getPosition(), m_elevatorEncoder.getVelocity());
  }
  
  public void setGoal(double desiredPosition, double desiredVelocity){
    m_goal = new TrapezoidProfile.State(desiredPosition, desiredVelocity);
    goToSetpoint();
  }

  private void goToSetpoint() {
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
    double m_output = MathUtil.clamp(m_setpoint.position, 0, 26);
    m_elevatorController.setReference(m_output, ControlType.kPosition);
  }
  
  public boolean atSetpoint() {
    return MathUtil.isNear(m_goal.position, getPosition(), 0.1);
  }

  //RESET/ZERO METHODS
  public void resetEncoder() {
    m_elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {  
    logData();
  }

  private void logData(){
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator output", m_elevatorMaster.getAppliedOutput());
    SmartDashboard.putNumber("Elevator goal position", m_goal.position);
    SmartDashboard.putNumber("Elevator setpoint position", m_setpoint.position);   
    SmartDashboard.putBoolean("Elevator setpoint reached", atSetpoint());
  }

  private void configMotors(){
    m_elevatorMaster.restoreFactoryDefaults();
    m_elevatorMaster.setInverted(false);
    m_elevatorMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorController.setP(ElevatorConstants.ELEVATOR_KP);
    m_elevatorController.setD(ElevatorConstants.ELEVATOR_KD);
    m_elevatorMaster.burnFlash();
  }
}
