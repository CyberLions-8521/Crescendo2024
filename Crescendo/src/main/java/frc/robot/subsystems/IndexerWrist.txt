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

public class IndexerWrist extends SubsystemBase {
  
  //CONSTRUCTOR
  IndexerWrist() {
    configMotors();
    resetEncoder();
  }

  //STATES
  public enum IndexerWristState{
        OFF,
        //JOG = MOTOR OUTPUT
        JOG,
        POSITION,
        ZERO
    }
  
  //SET STATE, JOGVALUE, AND SETPOINT
  private IndexerWristState m_state = IndexerWristState.OFF;
  private double jogValue = 0;
  private Rotation2d setpoint = new Rotation2d();
  
  //MOTOR OBJECT
  private CANSparkMax m_indexerWristMaster = new CANSparkMax(MotorConstants.INDEXER_WRIST_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_indexerWristEncoder = m_indexerWristMaster.getEncoder();
  
  //LIMIT SWITCH
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  //PID CONTROLLER
  private SparkPIDController m_indexerWristController = m_indexerWristMaster.getPIDController();

  //STATE METHODS
  public void setState(IndexerWristState m_state){
    this.m_state = m_state;
  }

  public IndexerWristState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_indexerWristMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(IndexerWristState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_indexerWristController.setReference(setpoint.getRotations(), ControlType.kPosition);
  }

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(IndexerWristState.POSITION);
  }

  public Rotation2d getSetpoint(){
    return setpoint;
  }

  //ZERO METHODS
  public void resetEncoder(){
    m_indexerWristEncoder.setPosition(0);
  }

  public void zero(){
    //IF LIMIT SWITCH IS NOT CLICKED --> GO BACKWARDS
    if(!m_limitSwitch.get()){
      setJogValue(-1);
    }
    else{
      setState(IndexerWristState.OFF);
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
    SmartDashboard.putString("indexer Wrist State", getState().toString());
    SmartDashboard.putNumber("Indexer Wrist Setpoint", getSetpoint().getDegrees());
  }

  public void configMotors(){
    m_indexerWristMaster.restoreFactoryDefaults();
    m_indexerWristMaster.setInverted(false);
    m_indexerWristMaster.setIdleMode(IdleMode.kBrake);
    m_indexerWristMaster.setSmartCurrentLimit(40, 40);
    
     
    m_indexerWristController.setP(IndexerWristConstants.INDEXER_WRIST_KP);
    m_indexerWristController.setD(IndexerWristConstants.INDEXER_WRIST_KD);

    m_indexerWristController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_indexerWristController.setSmartMotionMaxAccel(IndexerWristConstants.MAX_ACCELERATION, 0);
    m_indexerWristController.setSmartMotionMaxVelocity(IndexerWristConstants.MAX_VELOCITY, 0);
  }
}
