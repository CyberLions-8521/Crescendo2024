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

public class Hood extends SubsystemBase {
  
  //CONSTRUCTOR
  Hood() {
    configMotors();
    resetEncoder();
  }

  //STATES
  public enum hoodState{
        OFF,
        //JOG = MOTOR OUTPUT
        JOG,
        POSITION,
        ZERO
    }
  
  //SET STATE, JOGVALUE, AND SETPOINT
  private hoodState m_state = hoodState.OFF;
  private double jogValue = 0;
  private Rotation2d setpoint = new Rotation2d();
  
  //MOTOR OBJECT
  private CANSparkMax m_hoodMaster = new CANSparkMax(MotorConstants.HOOD_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_hoodEncoder = m_hoodMaster.getEncoder();
  
  //LIMIT SWITCH
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  //PID CONTROLLER
  private SparkPIDController m_hoodController = m_hoodMaster.getPIDController();

  //STATE METHODS
  public void setState(hoodState m_state){
    this.m_state = m_state;
  }

  public hoodState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_hoodMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(hoodState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_hoodController.setReference(setpoint.getRotations(), ControlType.kPosition);
  }

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(hoodState.POSITION);
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
      setState(hoodState.OFF);
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
    SmartDashboard.putString("hood State", getState().toString());
    SmartDashboard.putNumber("hood Setpoint", getSetpoint().getDegrees());
  }

  public void configMotors(){
    m_hoodMaster.restoreFactoryDefaults();
    m_hoodMaster.setInverted(false);
    m_hoodMaster.setIdleMode(IdleMode.kBrake);
    m_hoodMaster.setSmartCurrentLimit(40, 40);
    
     
    m_hoodController.setP(HoodConstants.HOOD_KP);
    m_hoodController.setD(HoodConstants.HOOD_KD);

    m_hoodController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_hoodController.setSmartMotionMaxAccel(HoodConstants.MAX_ACCELERATION, 0);
    m_hoodController.setSmartMotionMaxVelocity(HoodConstants.MAX_VELOCITY, 0);    
  }
}
