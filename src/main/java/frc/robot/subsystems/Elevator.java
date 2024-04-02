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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Joint.JointState;
public class Elevator extends SubsystemBase {

  //CONSTRUCTOR
  private Elevator() {
    configMotors();
    resetEncoder();
    
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

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(200, 100));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
  
  //PID CONTROLLER OBJECT
  private SparkPIDController m_elevatorController = m_elevatorMaster.getPIDController();

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

  public void setGoal(double desiredPosition, double desiredVelocity){
    m_goal = new TrapezoidProfile.State(desiredPosition, desiredVelocity);
    setState(ElevatorState.SETPOINT);
  }

  public double getGoal(){
    return m_goal.position;
  }

  public double getPosition(){
    return m_elevatorEncoder.getPosition();
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
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
    double m_output = MathUtil.clamp(m_setpoint.position, 0, 26);
    m_elevatorController.setReference(m_output, ControlType.kPosition);
  }
  
  public boolean atSetpoint(){
    return MathUtil.isNear(m_goal.position, getPosition(), 0.1);
  }

  public void refreshSetpoint(){
    m_setpoint = new TrapezoidProfile.State(getPosition(), m_elevatorEncoder.getVelocity());
  }

  //GETTER METHODS
  public double getElevatorHeight(){
    return m_elevatorEncoder.getPosition();
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
        set(0.00348837 * getElevatorHeight() * 0.3);
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
    SmartDashboard.putString("Elevator State", m_state.toString());
    SmartDashboard.putNumber("Elevator Position", getElevatorHeight());
    SmartDashboard.putNumber("Elevator output", m_elevatorMaster.getAppliedOutput());
    SmartDashboard.putNumber("Elevator goal position", m_goal.position);
    SmartDashboard.putNumber("Elevator setpoint position", m_setpoint.position);   
    SmartDashboard.putBoolean("Elevator setpoint reached", atSetpoint());
  }

  /*public void configElevatorPID(){
    //SET PID VALUES
    //m_elevatorController.setP(ElevatorConstants.ELEVATOR_KP);
    //m_elevatorController.setFF(ElevatorConstants.ELEVATOR_KFF);
    //m_elevatorController.setD(ElevatorConstants.ELEVATOR_KD);

    m_elevatorController.setP(SmartDashboard.getNumber("Elevator kP", Constants.ELevatorConstants.ELEVATOR_KP));
    m_elevatorController.setD(SmartDashboard.getNumber("Elevator kD", 0));
  }*/

  public void configMotors(){
    //RESTORE FACTORY DEFAULT
    m_elevatorMaster.restoreFactoryDefaults();

    //SET INVERSION
    m_elevatorMaster.setInverted(false);
    
    //SET IDLE MODE
    m_elevatorMaster.setIdleMode(IdleMode.kBrake);

    m_elevatorController.setP(Constants.ElevatorConstants.ELEVATOR_KP);
    m_elevatorController.setD(Constants.ElevatorConstants.ELEVATOR_KD);

    m_elevatorMaster.burnFlash();
  }
}
