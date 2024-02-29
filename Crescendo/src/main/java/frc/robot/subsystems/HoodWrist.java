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

public class HoodWrist extends SubsystemBase {
  
  //CONSTRUCTOR
  HoodWrist() {
    configMotors();
    resetEncoder();
  }

  //STATES
  public enum HoodWristState{
        OFF,
        //JOG = MOTOR OUTPUT
        JOG,
        POSITION,
        ZERO
    }
  
  //SET STATE, JOGVALUE, AND SETPOINT
  private HoodWristState m_state = HoodWristState.OFF;
  private double jogValue = 0;
  private Rotation2d setpoint = new Rotation2d();
  
  //MOTOR OBJECT
  private CANSparkMax m_hoodWristMaster = new CANSparkMax(MotorConstants.HOOD_WRIST_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_hoodEncoder = m_hoodWristMaster.getEncoder();
  
  //LIMIT SWITCH
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  //PID CONTROLLER
  private SparkPIDController m_hoodController = m_hoodWristMaster.getPIDController();

  //STATE METHODS
  public void setState(HoodWristState m_state){
    this.m_state = m_state;
  }

  public HoodWristState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_hoodWristMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(HoodWristState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_hoodController.setReference(setpoint.getRotations(), ControlType.kPosition);
  }

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(HoodWristState.POSITION);
  }

  public Rotation2d getSetpoint(){
    return setpoint;
  }

  //ZERO METHODS
  public void resetEncoder(){
    m_hoodEncoder.setPosition(0);
  }

  public void zero(){
    //IF LIMIT SWITCH IS NOT CLICKED --> GO BACKWARDS
    if(!m_limitSwitch.get()){
      setJogValue(-1);
    }
    else{
      setState(HoodWristState.OFF);
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
    SmartDashboard.putString("hood wrist State", getState().toString());
    SmartDashboard.putNumber("hood wrist Setpoint", getSetpoint().getDegrees());
  }

  public void configMotors(){
    m_hoodWristMaster.restoreFactoryDefaults();
    m_hoodWristMaster.setInverted(false);
    m_hoodWristMaster.setIdleMode(IdleMode.kBrake);
    m_hoodWristMaster.setSmartCurrentLimit(40, 40);
    
     
    m_hoodController.setP(HoodWristConstants.HOOD_WRIST_KP);
    m_hoodController.setD(HoodWristConstants.HOOD_WRIST_KD);

    m_hoodController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_hoodController.setSmartMotionMaxAccel(HoodWristConstants.MAX_ACCELERATION, 0);
    m_hoodController.setSmartMotionMaxVelocity(HoodWristConstants.MAX_VELOCITY, 0);    
  }
}
