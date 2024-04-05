package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ToasterConstants;

public class Toaster extends SubsystemBase {
  
  //CONSTRUCTOR
  public Toaster() {
    configMotors();
  }

  //STATES
  // public enum ToasterState{
  //   OFF,
  //   INTAKE,
  //   SPEAKER_SHOOT,
  //   AMP_SHOOT;
  // }

  //INSTANCE
  // private static Toaster m_instance = new Toaster();
  
  //SET STATE TO OFF
  // private ToasterState m_state = ToasterState.OFF;
  
  //MOTOR OBJECTS
  private final CANSparkMax m_toasterRight = new CANSparkMax(MotorConstants.TOASTER_RIGHT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_toasterLeft = new CANSparkMax(MotorConstants.TOASTER_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_holder = new CANSparkMax(MotorConstants.HOLDER_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder m_toasterEncoder = m_toasterRight.getEncoder();
  private final Timer m_timer = new Timer();

  //GET INSTANCE
  // public static Toaster getInstance(){
  //   return m_instance;
  // }

  //STATE METHODS
  // public void setState(ToasterState m_state){
  //   this.m_state = m_state;
  // }

  // public ToasterState getState(){
  //   return m_state;
  // }

  //SET MOTOR OUTPUT METHODS
  private void setSpeed(double intakeValue, double holderValue){
    // m_toasterRight.set(intakeValue);
    // m_toasterLeft.set(intakeValue);
    m_holder.set(holderValue);
    setShooterSpeed(intakeValue);
    // setHolderSpeed(holderValue);
  }

  private void setShooterSpeed(double intakeValue){
    m_toasterRight.set(intakeValue);
    m_toasterLeft.set(intakeValue);
  }

  // public void setHolderSpeed(double holderValue){
  //   m_holder.set(holderValue);
  // }

  private double getRPM(){
    return m_toasterEncoder.getVelocity();
  }

  private boolean atShootingSpeed(){
    return (getRPM() >= ToasterConstants.kShootingSpeed);
  }

  @Override
  public void periodic() {
    // switch(m_state){
    //   case OFF:
    //     //setSpeed(0, 0);
    //     m_toasterRight.setVoltage(0);
    //     m_toasterLeft.setVoltage(0);
    //     m_holder.set(0);
    //     m_timer.stop();
    //     m_timer.reset();
    //     break;
    //   case INTAKE:
    //     // setShooterSpeed(ToasterConstants.intakeSpeed);
    //     // m_holder.set(ToasterConstants.intakeSpeed);

    //     // equivalent call to
    //     setSpeed(ToasterConstants.intakeSpeed, ToasterConstants.intakeSpeed);
    //     break;
      // case SPEAKER_SHOOT:
      //   m_timer.start();
      //   setShooterSpeed(ToasterConstants.SpeakerShooterSpeed);
      //   if (m_timer.get() > ToasterConstants.waitTime){
      //     m_holder.set(ToasterConstants.SpeakerHolderSpeed);
      //   }
      //   break;
    //   case AMP_SHOOT:
    //     setSpeed(ToasterConstants.AmpShooterSpeed,ToasterConstants.AmpHolderSpeed);
    // }     
    logData();
  }

  public Command ToasterOffCmd() {
    return this.run(() -> {
      m_toasterRight.setVoltage(0);
      m_toasterLeft.setVoltage(0);
      m_holder.set(0);
    });
  }

  public Command ToasterIntakeCmd() {
    return this.run(() -> setSpeed(ToasterConstants.intakeSpeed, ToasterConstants.intakeSpeed));
  }

  public Command ToasterSpeakerShootCmd() {
    return this.run(() -> setShooterSpeed(ToasterConstants.SpeakerShooterSpeed))
      .andThen(new WaitCommand(ToasterConstants.waitTime))
      .andThen(this.run(() -> m_holder.set(ToasterConstants.SpeakerHolderSpeed)));
  }

  public Command ToasterAmpShootCmd() {
    return this.run(() -> setSpeed(ToasterConstants.AmpShooterSpeed, ToasterConstants.AmpHolderSpeed));
  }




  private void logData(){
    // SmartDashboard.putString("toaster State", getState().toString());
    SmartDashboard.putNumber("Current Draw", m_toasterLeft.getOutputCurrent());
    SmartDashboard.putNumber("Timer", m_timer.get());

    SmartDashboard.putNumber("Toaster RPM", getRPM());
    SmartDashboard.putBoolean("Shooting Correct Speed", atShootingSpeed());
  }

  private void configMotors(){
    
    //RESTORE FACTORY DEFAULT
    m_toasterRight.restoreFactoryDefaults();
    m_toasterLeft.restoreFactoryDefaults();
    m_holder.restoreFactoryDefaults();

    //SET INVERSION
    m_toasterRight.setInverted(false);
    m_toasterLeft.setInverted(true);
    m_holder.setInverted(false);

    //SET IDLE MODE
    m_toasterRight.setIdleMode(IdleMode.kCoast);
    m_toasterLeft.setIdleMode(m_toasterRight.getIdleMode());
    m_holder.setIdleMode(IdleMode.kBrake);

    m_toasterRight.setOpenLoopRampRate(0.1);
    m_toasterLeft.setOpenLoopRampRate(0.1);

    //SET SMART CURRENT LIMIT
    m_toasterRight.setSmartCurrentLimit(100, 100);
    m_toasterLeft.setSmartCurrentLimit(100, 100);
    m_holder.setSmartCurrentLimit(100, 100);

    m_toasterRight.burnFlash();
    m_toasterLeft.burnFlash();
    m_holder.burnFlash();
  }
}