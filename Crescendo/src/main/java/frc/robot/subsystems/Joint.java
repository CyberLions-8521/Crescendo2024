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

public class Joint extends SubsystemBase {
  
  //CONSTRUCTOR
  private Joint() {
    configMotors();
    configJointPID();
    resetEncoder();
  }

  //STATES
  public enum JointState{
        OFF,
        POSITION,
        ZERO
    }

  //INSTANCE
  private static Joint m_instance = new Joint();
  
  //SET STATE
  private JointState m_state = JointState.OFF;
  
  //MOTOR OBJECTS
  private CANSparkMax m_jointRight = new CANSparkMax(MotorConstants.JOINT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_jointLeft = new CANSparkMax(MotorConstants.JOINT_LEFT_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_jointRightEncoder = m_jointRight.getEncoder();
  private RelativeEncoder m_jointLeftEncoder = m_jointLeft.getEncoder();

  //LIMIT SWITCH OBJECT
  private DigitalInput m_limitSwitch = new DigitalInput(4);

  //MOTOR CONTROLLER OBJECT
  private SparkPIDController m_JointController = m_jointRight.getPIDController();
  
  //JOG VALUE & SETPOINT
  private Rotation2d setpoint;

  //GET INSTANCE
  public static Joint getInstance(){
    return m_instance;
  }

  //STATE OBJECTS
  public void setState(JointState m_state){
    this.m_state = m_state;
  }

  public JointState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_jointRight.set(value);
    m_jointLeft.set(value);
  }

  //SETPOINT METHODS
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

  public double getPosition(){
    return m_jointLeftEncoder.getPosition();
  }

  //ZERO/RESET METHODS
  public void resetEncoder(){
    m_jointRightEncoder.setPosition(0);
    m_jointLeftEncoder.setPosition(0);
  }

  public void zero(){
    if(!m_limitSwitch.get()){
      set(1);
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
    SmartDashboard.putNumber("Joint Setpoint", getSetpoint().getRotations());
    SmartDashboard.putNumber("Joint Position", getPosition());
  }

  public void configJointPID(){
    m_JointController.setP(JointConstants.JOINT_KP);
    m_JointController.setD(JointConstants.JOINT_KD);
  }

  public void configMotors(){
    //RESTORE FACTORY DEFAULT
    m_jointRight.restoreFactoryDefaults();
    m_jointLeft.restoreFactoryDefaults();

    //INVERSION
    m_jointRight.setInverted(true);
    m_jointLeft.setInverted(!m_jointRight.getInverted());

    //IDLE MODE
    m_jointRight.setIdleMode(IdleMode.kBrake);
    m_jointLeft.setIdleMode(m_jointRight.getIdleMode());

    //SET SMART CURRENT LIMIT
    m_jointRight.setSmartCurrentLimit(40, 40);
    m_jointLeft.setSmartCurrentLimit(40, 40);

    //SET SMART MOTION 
    m_JointController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_JointController.setSmartMotionMaxAccel(JointConstants.MAX_ACCELERATION, 0);
    m_JointController.setSmartMotionMaxVelocity(JointConstants.MAX_VELOCITY, 0);    
  }
}
