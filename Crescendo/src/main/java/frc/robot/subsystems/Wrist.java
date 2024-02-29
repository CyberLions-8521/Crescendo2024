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
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Wrist extends SubsystemBase {
  
  //CONSTRUCTOR
  Wrist() {
    configMotors();
    resetEncoder();
  }

  //STATES
  public enum WristState{
        OFF,
        //JOG = MOTOR OUTPUT
        JOG,
        POSITION,
        ZERO
    }
  
  //SET STATE, JOGVALUE, AND SETPOINT
  private WristState m_state = WristState.OFF;
  private double jogValue = 0;
  private Rotation2d setpoint = new Rotation2d();
  
  //MOTOR OBJECT
  private CANSparkMax m_wristMaster = new CANSparkMax(MotorConstants.WRIST_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_wristEncoder = m_wristMaster.getEncoder();
  
  //LIMIT SWITCH
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  //PID CONTROLLER
  private SparkPIDController m_wristController = m_wristMaster.getPIDController();

  //STATE METHODS
  public void setState(WristState m_state){
    this.m_state = m_state;
  }

  public WristState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_wristMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(WristState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_wristController.setReference(setpoint.getRotations(), ControlType.kPosition);
  }

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(WristState.POSITION);
  }

  public Rotation2d getSetpoint(){
    return setpoint;
  }

  //ZERO METHODS
  public void resetEncoder(){
    m_wristEncoder.setPosition(0);
  }

  public void zero(){
    //IF LIMIT SWITCH IS NOT CLICKED --> GO BACKWARDS
    if(!m_limitSwitch.get()){
      setJogValue(-1);
    }
    else{
      setState(WristState.OFF);
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
    m_wristMaster.restoreFactoryDefaults();
    m_wristMaster.setInverted(false);
    m_wristMaster.setIdleMode(IdleMode.kBrake);
    m_wristMaster.setSmartCurrentLimit(40, 40);
    
     
    m_wristController.setP(WristConstants.WRIST_KP);
    m_wristController.setD(WristConstants.WRIST_KD);

    m_wristController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_wristController.setSmartMotionMaxAccel(WristConstants.MAX_ACCELERATION, 0);
    m_wristController.setSmartMotionMaxVelocity(WristConstants.MAX_VELOCITY, 0);
  }
}
