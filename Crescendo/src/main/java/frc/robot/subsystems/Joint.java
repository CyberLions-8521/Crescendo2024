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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Joint extends SubsystemBase {
  
  //CONSTRUCTOR
  private Joint() {
    configMotors();
    rezero();
    SmartDashboard.putNumber("Joint kP", m_jointControllerLeft.getP());
    SmartDashboard.putNumber("Joint kd", m_jointControllerLeft.getD());
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
  private SparkPIDController m_jointControllerLeft = m_jointLeft.getPIDController();

  //TRAPEZOID PROFILE OBJECT
  //500,250

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(550, 150));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
  
  //JOG VALUE & SETPOINT
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
    setState(JointState.POSITION);
  }

  public double getGoal(){
    return m_goal.position;
  }

  //SET MOTOR OUTPUT METHODS
  public void set(double value){
    m_jointLeft.set(value);
  }

  public void setJog(double jog){
    jogValue = jog;
    setState(JointState.JOG);
  }

  //SETPOINT METHODS
  public void goToSetpoint(){
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
    double output = MathUtil.clamp(m_setpoint.position, 0, 25);
    m_jointControllerLeft.setReference(output, ControlType.kPosition);
  }

  public boolean atSetpoint(){
    return MathUtil.isNear(m_goal.position, getPosition(), 0.5);
  }

  public void refreshSetpoint(){
    m_setpoint = new TrapezoidProfile.State(getPosition(), m_jointEncoderLeft.getVelocity());
  }

  public double getPosition(){
    return (m_jointEncoderLeft.getPosition());
  }

  public boolean getFollower(){
    return (m_jointRight.isFollower());
  }

  public void zero(){
    if(getPosition() > 0){
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
        set(0.000806452 * getPosition());
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
    m_jointRight.follow(m_jointLeft, true);
  }

  public void logData(){
    SmartDashboard.putString("Joint State", getState().toString());

    SmartDashboard.putNumber("Joint Goal Position", m_goal.position);
    SmartDashboard.putNumber("Joint Goal Setpoint", m_setpoint.position);

    SmartDashboard.putNumber("joint velocity", m_jointEncoderLeft.getVelocity());

    SmartDashboard.putNumber("Joint Left", m_jointLeft.get());
    SmartDashboard.putNumber("Joint Right", m_jointRight.get());

    SmartDashboard.putBoolean("At setpoint", atSetpoint());

    SmartDashboard.putNumber("Joint Position", m_jointEncoderLeft.getPosition());
  }


  public void configMotors(){

    //RESTORE FACTORY DEFAULT
    m_jointRight.restoreFactoryDefaults();
    m_jointLeft.restoreFactoryDefaults();

    m_jointControllerLeft.setP(JointConstants.JOINT_KP);
    m_jointControllerLeft.setD(JointConstants.JOINT_KD);

    //INVERSION
    /*m_jointRight.setInverted(true);
    m_jointLeft.setInverted(!m_jointRight.getInverted());*/

    //IDLE MODE
    m_jointRight.setIdleMode(IdleMode.kBrake);
    m_jointLeft.setIdleMode(m_jointRight.getIdleMode());

    //SET SMART CURRENT LIMIT
    m_jointRight.setSmartCurrentLimit(40, 40);
    m_jointLeft.setSmartCurrentLimit(40, 40);

    m_jointRight.burnFlash();
    m_jointLeft.burnFlash();
  }
}
