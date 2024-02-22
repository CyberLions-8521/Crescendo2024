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
  
  //Constructor & Instance
  public Toaster() {
    configMotors();
  }

  //Enum
  public enum ToasterState{
        OFF,
        SHOOT,
        INTAKE
    }
  
  private ToasterState m_state = ToasterState.OFF;
  
  //Objects
  private CANSparkMax m_ToasterRight = new CANSparkMax(MotorConstants.TOASTER_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_ToasterLeft = new CANSparkMax(MotorConstants.TOASTER_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_Holder = new CANSparkMax(MotorConstants.HOLDER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_ToasterEncoder = m_ToasterRight.getEncoder();
  private SparkPIDController m_ToasterController = m_ToasterRight.getPIDController();
  
  private double RPM = 0;


  public void setState(ToasterState m_state){
    this.m_state = m_state;
  }

  public ToasterState getState(){
    return m_state;
  }

  public void set(double value){
    m_ToasterRight.set(value);
    m_ToasterLeft.set(value);
  }
  
  public void shoot(){
    m_ToasterController.setReference(RPM, ControlType.kVelocity);
  }

  public void setRPM(double RPM){
    this.RPM = RPM;
    setState(ToasterState.SHOOT);
  }

  public void resetEncoder(){
    m_ToasterEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    switch(m_state){
      case OFF:
        set(0);
        break;
      case SHOOT:
        break;
      case INTAKE:
        break;
    }     
  }

  public void logData(){
    SmartDashboard.putString("Toaster State", getState().toString());
  }

  public void configMotors(){
    m_ToasterRight.restoreFactoryDefaults();
    m_ToasterRight.setInverted(false);
    m_ToasterRight.setIdleMode(IdleMode.kBrake);
    m_ToasterRight.setSmartCurrentLimit(40, 40);

    m_ToasterLeft.restoreFactoryDefaults();
    m_ToasterLeft.setInverted(!m_ToasterRight.getInverted());
    m_ToasterLeft.setIdleMode(IdleMode.kBrake);
    m_ToasterLeft.setSmartCurrentLimit(40, 40);

    m_Holder.restoreFactoryDefaults();
    m_Holder.setInverted(false);
    m_Holder.setIdleMode(IdleMode.kBrake);
    m_Holder.setSmartCurrentLimit(40, 40);

    m_ToasterController.setP(ToasterConstants.TOASTER_KD);
    m_ToasterController.setD(ToasterConstants.TOASTER_KD);

    m_ToasterController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_ToasterController.setSmartMotionMaxAccel(ToasterConstants.MAX_ACCELERATION, 0);
    m_ToasterController.setSmartMotionMaxVelocity(ToasterConstants.MAX_VELOCITY, 0);    
  }
}