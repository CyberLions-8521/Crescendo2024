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
import frc.robot.subsystems.Toaster.ToasterState;

public class Toaster extends SubsystemBase {
  
  //CONSTRUCTOR
  public Toaster() {
    configMotors();
  }

  //STATES
  public enum ToasterState{
        OFF,
        SHOOT,
        INTAKE
    }
  
  //SET STATE TO OFF
  private ToasterState m_state = ToasterState.OFF;
  
  //MOTOR OBJECTS
  private CANSparkMax m_toasterRight = new CANSparkMax(MotorConstants.TOASTER_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_toasterLeft = new CANSparkMax(MotorConstants.TOASTER_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_holder = new CANSparkMax(MotorConstants.HOLDER_MOTOR, MotorType.kBrushless);
  
  //ENCODER OBJECT
  private RelativeEncoder m_toasterEncoder = m_toasterRight.getEncoder();

  //PID CONTROLLER
  private SparkPIDController m_toasterController = m_toasterRight.getPIDController();
  
  private double RPM = 0;

  public void setState(ToasterState m_state){
    this.m_state = m_state;
  }

  public ToasterState getState(){
    return m_state;
  }

  public void set(double intakeValue, double holderValue){
    m_toasterRight.set(intakeValue);
    m_toasterLeft.set(intakeValue);
    m_holder.set(holderValue);
  }
  
  public void shoot(){
    m_toasterController.setReference(RPM, ControlType.kVelocity);
  }

  public void setRPM(double RPM){
    this.RPM = RPM;
    setState(ToasterState.SHOOT);
  }

  public void resetEncoder(){
    m_toasterEncoder.setPosition(0);
  }

  public void intake(){

    set(-0.8, -0.8);
    /*if(m_toasterLeft.getOutputCurrent() >= 0.01){
      
    }else{
      
    }*/
  }
  
  @Override
  public void periodic() {
    switch(m_state){
      case OFF:
        set(0, 0);
        break;
      case SHOOT:
        shoot();
        break;
      case INTAKE: 
        intake();  
        break;
    }     
  }

  public void logData(){
    SmartDashboard.putString("toaster State", getState().toString());
    SmartDashboard.putNumber("Current Draw", m_toasterLeft.getOutputCurrent());
  }

  public void configMotors(){
    m_toasterRight.restoreFactoryDefaults();
    m_toasterRight.setInverted(false);
    m_toasterRight.setIdleMode(IdleMode.kBrake);
    m_toasterRight.setSmartCurrentLimit(40, 40);

    m_toasterLeft.restoreFactoryDefaults();
    m_toasterLeft.setInverted(!m_toasterRight.getInverted());
    m_toasterLeft.setIdleMode(IdleMode.kBrake);
    m_toasterLeft.setSmartCurrentLimit(40, 40);

    m_holder.restoreFactoryDefaults();
    m_holder.setInverted(false);
    m_holder.setIdleMode(IdleMode.kBrake);
    m_holder.setSmartCurrentLimit(40, 40);

    m_toasterController.setP(ToasterConstants.TOASTER_KP);
    m_toasterController.setD(ToasterConstants.TOASTER_KD);

    m_toasterController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_toasterController.setSmartMotionMaxAccel(ToasterConstants.MAX_ACCELERATION, 0);
    m_toasterController.setSmartMotionMaxVelocity(ToasterConstants.MAX_VELOCITY, 0);    
  }
}