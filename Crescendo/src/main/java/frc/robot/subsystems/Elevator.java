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
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
public class Elevator extends SubsystemBase {

  public Elevator() {
    configMotors();
  }
 
  public enum ElevatorState{
    OFF,
    JOG,
    POSITION,
    ZERO
  }

  private ElevatorState m_state = ElevatorState.OFF;

  private CANSparkMax m_elevatorMaster = new CANSparkMax(MotorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_elevatorEncoder = m_elevatorMaster.getEncoder();
  private DigitalInput m_limitSwitch = new DigitalInput(0);
  private SparkPIDController m_elevatorController = m_elevatorMaster.getPIDController();
  
  private double jogValue = 0;
  private Rotation2d setpoint = new Rotation2d();

  public void setState(ElevatorState m_state){
    this.m_state = m_state;
  }
  
  public ElevatorState getState(){
    return m_state;
  }

  public void set(double value){
    m_elevatorMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(ElevatorState.JOG);
  }

  public void goToSetpoint(){
    m_elevatorController.setReference(setpoint.getRotations(), ControlType.kSmartMotion);
  }

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(ElevatorState.POSITION);
  }

  public Rotation2d getSetpoint(){
    return setpoint;
  }

  public void resetEncoder(){
    m_elevatorEncoder.setPosition(0);
  }

  public void zero(){
    if(!m_limitSwitch.get()){
      setJogValue(-1);
    }else{
      setState(ElevatorState.OFF);
      resetEncoder();
    }
  }

  @Override
  public void periodic() {
    switch(m_state){
      case OFF:
        set(0);
        break;
      case JOG:
        set(jogValue);
        break;
      case POSITION:
        goToSetpoint();
        break;
      case ZERO:
        zero();
        break;
    }   
  }

  public void logData(){
    SmartDashboard.putString("Wrist State", getState().toString());
    SmartDashboard.putNumber("Wrist Setpoint", getSetpoint().getDegrees());
  }

  public void configMotors(){
    m_elevatorMaster.restoreFactoryDefaults();
    m_elevatorMaster.setInverted(false);
    m_elevatorMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorMaster.setSmartCurrentLimit(40, 40);
    
     
    m_elevatorController.setP(ElevatorConstants.ELEVATOR_KP);
    m_elevatorController.setD(ElevatorConstants.ELEVATOR_KD);

    m_elevatorController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_elevatorController.setSmartMotionMaxAccel(ElevatorConstants.MAX_ACCELERATION, 0);
    m_elevatorController.setSmartMotionMaxVelocity(ElevatorConstants.MAX_VELOCITY, 0);    
  }
}
