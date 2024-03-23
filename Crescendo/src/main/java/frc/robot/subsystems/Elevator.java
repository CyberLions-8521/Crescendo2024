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
  private Elevator() {
    configMotors();
    resetEncoder();
    configElevatorPID();
    
    SmartDashboard.putNumber("Elevator kP", ElevatorConstants.ELEVATOR_KP);
    SmartDashboard.putNumber("Elevator kD", ElevatorConstants.ELEVATOR_KD);
  }
 
  //STATES ENUM
  public enum ElevatorState{
    OFF,
    SETPOINT,
    JOG,
    ZERO
  }

  //INSTANCE
  private static Elevator instance = new Elevator();

  //SETTING STATE TO DEFAULT
  private ElevatorState m_state = ElevatorState.OFF;

  //MOTOR OBJECTS
  private CANSparkMax m_elevatorMaster = new CANSparkMax(MotorConstants.ELEVATOR_MASTER_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_elevatorEncoder = m_elevatorMaster.getEncoder();

  private double jogValue;


  //PID CONTROLLER OBJECT
  private SparkPIDController m_elevatorController = m_elevatorMaster.getPIDController();
  
  //JOG VALUE AND SETPOINT
  private double setpoint;

  //INSTANCE
  public static Elevator getInstance(){
    return instance;
  }  

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

  public void setJog(double jogValue){
    this.jogValue = jogValue;
    setState(ElevatorState.JOG);
  }

  public double getJog(){
    return jogValue;
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    /*if(m_elevatorEncoder.getPosition() > 25){
      set(0);
    }*/
    m_elevatorController.setReference(setpoint, ControlType.kPosition);
    if(atSetpoint()){
      setState(ElevatorState.OFF);
    }
  }

  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
    setState(ElevatorState.SETPOINT);
  }

  public double getSetpoint(){
    return setpoint;
  }

  public boolean atSetpoint(){
    return Math.abs(setpoint - getElevatorHeight()) < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE;
  }

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
    if(m_elevatorEncoder.getPosition() > 0) {
      set(-0.2);
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
      case SETPOINT:
        goToSetpoint();
        break;
      case ZERO:
        zero();
        break;
    }   
    logData();
  }

  public void logData(){
    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    SmartDashboard.putString("Elevator State", m_state.toString());
    SmartDashboard.putNumber("Elevator Position", getElevatorHeight());
    // SmartDashboard.putBoolean("elevator Limit Switch", m_limitSwitch.get());
    SmartDashboard.putBoolean("Elevator at Position", atSetpoint());        
  }

  public void configElevatorPID(){
    //SET PID VALUES
    //m_elevatorController.setP(ElevatorConstants.ELEVATOR_KP);
    //m_elevatorController.setFF(ElevatorConstants.ELEVATOR_KFF);
    //m_elevatorController.setD(ElevatorConstants.ELEVATOR_KD);

    m_elevatorController.setP(SmartDashboard.getNumber("Elevator kP", 0));
    m_elevatorController.setD(SmartDashboard.getNumber("Elevator kD", 0));
  }

  public void configMotors(){
    //RESTORE FACTORY DEFAULT
    m_elevatorMaster.restoreFactoryDefaults();

    //SET INVERSION
    m_elevatorMaster.setInverted(false);
    
    //SET IDLE MODE
    m_elevatorMaster.setIdleMode(IdleMode.kBrake);

    //SET SMART CURRENT LIMIT
    m_elevatorMaster.setSmartCurrentLimit(40, 40);

    //SET SMART MOTION STRATEGIES
    m_elevatorController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_elevatorController.setSmartMotionMaxAccel(ElevatorConstants.MAX_ACCELERATION, 0);
    m_elevatorController.setSmartMotionMaxVelocity(ElevatorConstants.MAX_VELOCITY, 0);    
  }
}
