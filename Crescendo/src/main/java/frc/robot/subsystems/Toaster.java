package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;

public class Toaster extends SubsystemBase {
  
  //CONSTRUCTOR
  public Toaster() {
    configMotors();
  }

  //STATES
  public enum ToasterState{
        OFF,
        INTAKE,
        SPEAKER_SHOOT,
        AMP_SHOOT;
    }

  //INSTANCE
  private static Toaster m_instance = new Toaster();
  
  //SET STATE TO OFF
  private ToasterState m_state = ToasterState.OFF;
  
  //MOTOR OBJECTS
  private CANSparkMax m_toasterRight = new CANSparkMax(MotorConstants.TOASTER_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_toasterLeft = new CANSparkMax(MotorConstants.TOASTER_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_holder = new CANSparkMax(MotorConstants.HOLDER_MOTOR, MotorType.kBrushless);

    private RelativeEncoder m_toasterEncoder = m_toasterRight.getEncoder();


  Timer m_timer = new Timer();

  //GET INSTANCE
  public static Toaster getInstance(){
    return m_instance;
  }

  //STATE METHODS
  public void setState(ToasterState m_state){
    this.m_state = m_state;
  }

  public ToasterState getState(){
    return m_state;
  }

  //SET MOTOR OUTPUT METHODS
  public void setSpeed(double intakeValue, double holderValue){
    m_toasterRight.set(intakeValue);
    m_toasterLeft.set(intakeValue);
    m_holder.set(holderValue);
  }

  public void setShooterSpeed(double intakeValue){
    m_toasterRight.set(intakeValue);
    m_toasterLeft.set(intakeValue);
  }

  public void setHolderSpeed(double holderValue){
    m_holder.set(holderValue);
  }

  public double getRPM(){
    return m_toasterEncoder.getVelocity();
  }

  public boolean atShootingSpeed(){
    return (getRPM() >= 5000);
  }

  @Override
  public void periodic() {
    switch(m_state){
      case OFF:
        setSpeed(0, 0);
        m_timer.stop();
        m_timer.reset();
        break;
      case INTAKE:
        setShooterSpeed(ToasterConstants.intakeSpeed);
        setHolderSpeed(ToasterConstants.intakeSpeed);
        break;
      case SPEAKER_SHOOT:
        m_timer.start();
        setShooterSpeed(ToasterConstants.SpeakerShooterSpeed);
        if (m_timer.get() > ToasterConstants.waitTime){
          setHolderSpeed(ToasterConstants.SpeakerHolderSpeed);
        }
        break;
      case AMP_SHOOT:
        setSpeed(ToasterConstants.AmpShooterSpeed,ToasterConstants.AmpHolderSpeed);
    }     
    logData();
  }

  public void logData(){
    SmartDashboard.putString("toaster State", getState().toString());
    SmartDashboard.putNumber("Current Draw", m_toasterLeft.getOutputCurrent());
    SmartDashboard.putNumber("Timer", m_timer.get());

    SmartDashboard.putNumber("Toaster RPM", getRPM());
    SmartDashboard.putBoolean("Shooting Correct Speed", atShootingSpeed());
  }

  public void configMotors(){
    
    //RESTORE FACTORY DEFAULT
    m_toasterRight.restoreFactoryDefaults();
    m_toasterLeft.restoreFactoryDefaults();
    m_holder.restoreFactoryDefaults();

    //SET INVERSION
    m_toasterRight.setInverted(false);
    m_toasterLeft.setInverted(true);
    m_holder.setInverted(false);

    //SET IDLE MODE
    m_toasterRight.setIdleMode(IdleMode.kBrake);
    m_toasterLeft.setIdleMode(m_toasterRight.getIdleMode());
    m_holder.setIdleMode(IdleMode.kBrake);

    //SET SMART CURRENT LIMIT
    m_toasterRight.setSmartCurrentLimit(40, 40);
    m_toasterLeft.setSmartCurrentLimit(40, 40);
    m_holder.setSmartCurrentLimit(40, 40);

    m_toasterRight.burnFlash();
    m_toasterLeft.burnFlash();
    m_holder.burnFlash();
  }
}