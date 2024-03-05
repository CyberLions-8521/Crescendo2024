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

  Timer m_timer = new Timer();

  //ENCODER OBJECT
  private RelativeEncoder m_rightEncoder = m_toasterRight.getEncoder();

  //PID CONTROLLER
  private SparkPIDController m_toasterController = m_toasterRight.getPIDController();

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
        //setShooterSpeed(ToasterConstants.SpeakerShooterSpeed);
        //setShooterSpeed(0.5);
        setHolderSpeed(0.5);
        //if (m_timer.get() > 5){
          //setHolderSpeed(ToasterConstants.SpeakerHolderSpeed);
        //}
        // setHolderSpeed(ToasterConstants.SpeakerHolderSpeed);
        //(new RunCommand(() -> setShooterSpeed(ToasterConstants.SpeakerShooterSpeed)))
        //.alongWith( new WaitCommand(ToasterConstants.SpeakerWaitTime)
        //.andThen(new RunCommand(() -> setHolderSpeed(ToasterConstants.SpeakerHolderSpeed)))
        // /.alongWith( new WaitCommand(ToasterConstants.waitTime))
        //.andThen( new RunCommand(() -> setState(ToasterState.OFF)))
        //);
        break;
      case AMP_SHOOT:
        //setShooterSpeed(ToasterConstants.AmpShooterSpeed);
        // setHolderSpeed(ToasterConstants.AmpHolderSpeed);
        (new RunCommand(() -> setShooterSpeed(ToasterConstants.AmpShooterSpeed)))
        .alongWith( new WaitCommand(ToasterConstants.AmpWaitTime)
        .andThen(new RunCommand(() -> setHolderSpeed(ToasterConstants.AmpHolderSpeed)))
        .alongWith( new WaitCommand(ToasterConstants.waitTime))
        .andThen( new RunCommand(() -> setState(ToasterState.OFF)))
        );
    }     
    logData();
  }

  public void logData(){
    SmartDashboard.putString("toaster State", getState().toString());
    SmartDashboard.putNumber("Current Draw", m_toasterLeft.getOutputCurrent());
    SmartDashboard.putNumber("Timer", m_timer.get());
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

    //SET SMART MOTION
    m_toasterController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    m_toasterController.setSmartMotionMaxAccel(ToasterConstants.MAX_ACCELERATION, 0);
    m_toasterController.setSmartMotionMaxVelocity(ToasterConstants.MAX_VELOCITY, 0);    
  }
}