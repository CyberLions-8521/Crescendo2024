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
import frc.robot.subsystems.Joint.JointState;

public class Joint extends SubsystemBase {
  
  //Constructor & Instance
  Joint() {
    configMotors();
  }

  //Enum
  public enum JointState{
        OFF,
        JOG,
        POSITION,
        ZERO
    }
  
  private JointState m_state = JointState.OFF;
  
  //Objects
  private CANSparkMax m_JointMaster = new CANSparkMax(MotorConstants.JOINT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_JointEncoder = m_JointMaster.getEncoder();
  private DigitalInput m_limitSwitch = new DigitalInput(0);
  private SparkPIDController m_JointController = m_JointMaster.getPIDController();
  
  private double jogValue = 0;
  private Rotation2d setpoint = new Rotation2d();


  public void setState(JointState m_state){
    this.m_state = m_state;
  }

  public JointState getState(){
    return m_state;
  }

  public void set(double value){
    m_JointMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(JointState.JOG);
  }

  public void goToSetpoint(){
    m_JointController.setReference(setpoint.getRotations(), ControlType.kPosition);
  }

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(JointState.POSITION);
  }

  public Rotation2d getSetpoint(){
    return setpoint;
  }

  public void resetEncoder(){
    m_JointEncoder.setPosition(0);
  }

  public void zero(){
    if(!m_limitSwitch.get()){
      setJogValue(-1);
    }else{
    setState(JointState.OFF);
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
    SmartDashboard.putString("Joint State", getState().toString());
    SmartDashboard.putNumber("Joint Setpoint", getSetpoint().getDegrees());
  }

  public void configMotors(){
    m_JointMaster.restoreFactoryDefaults();
    m_JointMaster.setInverted(false);
    m_JointMaster.setIdleMode(IdleMode.kBrake);
    m_JointMaster.setSmartCurrentLimit(40, 40);
    
     
    m_JointController.setP(JointConstants.JOINT_KP);
    m_JointController.setD(JointConstants.JOINT_KD);

    m_JointController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_JointController.setSmartMotionMaxAccel(JointConstants.MAX_ACCELERATION, 0);
    m_JointController.setSmartMotionMaxVelocity(JointConstants.MAX_VELOCITY, 0);    
  }
}
