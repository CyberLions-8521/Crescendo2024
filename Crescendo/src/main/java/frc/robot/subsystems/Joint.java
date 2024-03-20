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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Joint extends SubsystemBase {
  
  //CONSTRUCTOR
  private Joint() {
    configMotors();
    configJointPID();
    SmartDashboard.putNumber("Joint kP", 0);
    SmartDashboard.putNumber("Joint kd", 0);
  }

  //STATES
  public enum JointState{
        OFF,
        POSITION,
        JOG,
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
  //public final AbsoluteEncoder m_jointEncoder = m_jointRight.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  public RelativeEncoder m_jointEncoderRight = m_jointRight.getEncoder();
  public RelativeEncoder m_jointEncoderLeft = m_jointLeft.getEncoder();

  //MOTOR CONTROLLER OBJECT
  private SparkPIDController m_jointControllerRight = m_jointRight.getPIDController();
  private SparkPIDController m_jointControllerLeft = m_jointLeft.getPIDController();

  //TRAPEZOID PROFILE OBJECT
  //1.75, 0.75
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.75, 0.25));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  
  //JOG VALUE & SETPOINT
  private Rotation2d setpoint = new Rotation2d();
  private double jogValue = 0;

  //-----------------------

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

  public void setGoal(double desiredPosition, double desiredVelocity){
    m_goal = new TrapezoidProfile.State(desiredPosition, desiredVelocity);
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_jointRight.set(value);
    m_jointLeft.set(value);
  }

  public void setJog(double jog){
    jogValue = jog;
    setState(JointState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    //m_jointControllerRight.setReference(setpoint.getRotations(), ControlType.kPosition);
    //m_jointControllerLeft.setReference(setpoint.getRotations(), ControlType.kPosition);
    m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
    m_setpoint = m_profile.calculate(10, m_setpoint, m_goal);
    m_jointControllerRight.setReference(m_setpoint.position, ControlType.kPosition);
    m_jointControllerLeft.setReference(m_setpoint.position, ControlType.kPosition);
  }

  /*public boolean atSetpoint(){
    return Math.abs(setpoint - m_jointLeftEncoder.getPosition()) < JointConstants.JOINT_TOLERANCE.getRotations();
  }*/

  public void setSetpoint(Rotation2d setpoint){
    this.setpoint = setpoint;
    setState(JointState.POSITION);
  }

  public Rotation2d getSetpoint(){
    return setpoint;
  }

  public double getPosition(){
    return m_jointEncoderRight.getPosition();
  }

  public void zero(){
    if(getPosition() > 0.02){
      set(-0.15);
    }
    else{
     setState(JointState.OFF);
    }
  }

  public void rezero(){
    m_jointEncoderRight.setPosition(0);
    m_jointEncoderLeft.setPosition(0);
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
    logData();  
  }

  public void logData(){
    SmartDashboard.putString("Joint State", getState().toString());
    SmartDashboard.putNumber("Joint Setpoint", getSetpoint().getRotations());
    SmartDashboard.putNumber("Joint Position", getPosition());

    SmartDashboard.putNumber("Joint Goal Position", m_goal.position);
    SmartDashboard.putNumber("Joint Goal Setpoint", m_setpoint.position);
  }

  public void configJointPID(){
    m_jointControllerRight.setP(SmartDashboard.getNumber("Joint kP", 0));
    m_jointControllerRight.setD(SmartDashboard.getNumber("Joint kd", 0));

    m_jointControllerLeft.setP(SmartDashboard.getNumber("Joint kP", 0));
    m_jointControllerLeft.setD(SmartDashboard.getNumber("Joint kd", 0));
  }

  public void configMotors(){
    m_jointControllerRight.setP(JointConstants.JOINT_KP);
    m_jointControllerRight.setD(JointConstants.JOINT_KD);

    m_jointControllerLeft.setP(JointConstants.JOINT_KP);
    m_jointControllerLeft.setD(JointConstants.JOINT_KD);
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

    //m_jointLeft.follow(m_jointRight);

    //SET SMART MOTION 
    // m_JointController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    // m_JointController.setSmartMotionMaxAccel(JointConstants.MAX_ACCELERATION, 0);
    // m_JointController.setSmartMotionMaxVelocity(JointConstants.MAX_VELOCITY, 0);    
  }
}
