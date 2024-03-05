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
        POSITION,
        ZERO
    }
  
  //INSTANCE
  private static IndexerWrist m_instance = new IndexerWrist();

  //STATE
  private IndexerWristState m_state = IndexerWristState.OFF;

  //SETPOINT
  private double setpoint;
  
  //MOTOR OBJECT
  private CANSparkMax m_indexerWristMaster = new CANSparkMax(MotorConstants.INDEXER_WRIST_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_indexerWristEncoder = m_indexerWristMaster.getEncoder();
  
  //LIMIT SWITCH
  private DigitalInput m_limitSwitch = new DigitalInput(1);

  //PID CONTROLLER
  private SparkPIDController m_indexerWristController = m_indexerWristMaster.getPIDController();

  //GET INSTANCE
  public static IndexerWrist getInstance(){
    return m_instance;
  }

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

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_indexerWristController.setReference(setpoint, ControlType.kPosition);
  }

  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
    setState(IndexerWristState.POSITION);
  }

  public double getSetpoint(){
    return setpoint;
  }

  //ZERO METHODS
  public void resetEncoder(){
    m_indexerWristEncoder.setPosition(0);
  }

  public void zero(){
    //IF LIMIT SWITCH IS NOT CLICKED --> GO BACKWARDS
    if(!m_limitSwitch.get()){
      set(-1);
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
    SmartDashboard.putNumber("Indexer Wrist Setpoint", getSetpoint());
  }

  public void configMotors(){
    m_indexerWristMaster.restoreFactoryDefaults();
    m_indexerWristMaster.setInverted(false);
    m_indexerWristMaster.setIdleMode(IdleMode.kBrake);
    m_indexerWristMaster.setSmartCurrentLimit(40, 40);
    
     
    //m_indexerWristController.setP(IndexerWristConstants.INDEXER_WRIST_KP);
    //m_indexerWristController.setD(IndexerWristConstants.INDEXER_WRIST_KD);

    m_indexerWristController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
   // m_indexerWristController.setSmartMotionMaxAccel(IndexerWristConstants.MAX_ACCELERATION, 0);
    //m_indexerWristController.setSmartMotionMaxVelocity(IndexerWristConstants.MAX_VELOCITY, 0);
  }
}
