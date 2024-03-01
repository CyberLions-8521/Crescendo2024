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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
public class Elevator extends SubsystemBase {

  //CONSTRUCTOR
  public Elevator() {
    configMotors();
  }
 
  //STATES ENUM
  public enum ElevatorState{
    OFF,
    JOG,
    SETPOINT,
    POSITION,
    ZERO
  }

  //SETTING STATE TO DEFAULT
  private ElevatorState m_state = ElevatorState.OFF;

  //MOTOR OBJECTS
  private CANSparkMax m_elevatorMaster = new CANSparkMax(MotorConstants.ELEVATOR_MASTER_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_elevatorSlave = new CANSparkMax(MotorConstants.ELEVATOR_SLAVE_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_elevatorEncoder = m_elevatorMaster.getEncoder();

  //LIMIT SWITCH OBJECT
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  //PID CONTROLLER OBJECT
  private SparkPIDController m_elevatorController = m_elevatorMaster.getPIDController();
  
  //JOG VALUE AND SETPOINT
  private double jogValue = 0;
  private double setpoint;

  //STATE METHODS
  public void setState(ElevatorState m_state){
    this.m_state = m_state;
  }
  
  public ElevatorState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_elevatorMaster.set(value);
  }

  public void setJogValue(double jogValue){
    this.jogValue = jogValue;
    setState(ElevatorState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_elevatorController.setReference(setpoint, ControlType.kSmartMotion);
  }

  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
    setState(ElevatorState.POSITION);
  }

  public double getSetpoint(){
    return setpoint;
  }

  public boolean atSetpoint(){
    return Math.abs(setpoint - getElevatorHeight()) < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE;
  }

  public boolean setElevatorHeight(double setpoint){
        this.setpoint = setpoint;
        setState(ElevatorState.SETPOINT);
        configMotors();
        m_elevatorController.setReference(setpoint, ControlType.kSmartMotion);

        return atSetpoint();
  }

  /*public boolean atZero(){
        return absoluteEncoder.getAbsolutePosition() == ElevatorConstants.ELEVATOR_ZERO_HEIGHT;
  }*/

  //GETTER METHODS
  public double getElevatorHeight(){
    return m_elevatorEncoder.getPosition();
  }

  private double getError(){
    return Math.abs(setpoint - getElevatorHeight());
  }

  //RESET/ZERO METHODS
  public void resetEncoder(){
    m_elevatorEncoder.setPosition(0);
  }

  public void zero(){
    if(!m_limitSwitch.get()) {
      setJogValue(-1);
    }
    else {
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
    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    SmartDashboard.putString("Elevator State", m_state.toString());
    SmartDashboard.putNumber("Elevator Position", m_elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
  }

  public void configElevatorPID(){
    //SET PID VALUES
    m_elevatorController.setP(ElevatorConstants.ELEVATOR_KP);
    m_elevatorController.setFF(ElevatorConstants.ELEVATOR_KFF);
    m_elevatorController.setD(ElevatorConstants.ELEVATOR_KD);
  }

  public void configMotors(){
    //FOLLOW METHOD
    m_elevatorSlave.follow(m_elevatorMaster);
    
    //RESTORE FACTORY DEFAULT
    m_elevatorMaster.restoreFactoryDefaults();
    m_elevatorSlave.restoreFactoryDefaults();

    //SET INVERSION
    m_elevatorMaster.setInverted(false);
    m_elevatorSlave.setIdleMode(m_elevatorMaster.getIdleMode());
    
    //SET IDLE MODE
    m_elevatorMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorSlave.setIdleMode(m_elevatorMaster.getIdleMode());

    //SET SMART CURRENT LIMIT
    m_elevatorMaster.setSmartCurrentLimit(40, 40);
    m_elevatorSlave.setSmartCurrentLimit(40,40);

    //SET SMART MOTION STRATEGIES
    m_elevatorController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_elevatorController.setSmartMotionMaxAccel(ElevatorConstants.MAX_ACCELERATION, 0);
    m_elevatorController.setSmartMotionMaxVelocity(ElevatorConstants.MAX_VELOCITY, 0);    
  }
}
