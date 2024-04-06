package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ToasterConstants;

public class Toaster extends SubsystemBase {
  //CONSTRUCTOR
  public Toaster() {
    configMotors();
  }

  //MOTOR OBJECTS
  private final CANSparkMax m_toasterRight = new CANSparkMax(MotorConstants.TOASTER_RIGHT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_toasterLeft = new CANSparkMax(MotorConstants.TOASTER_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_holder = new CANSparkMax(MotorConstants.HOLDER_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder m_toasterEncoder = m_toasterRight.getEncoder();
  private final Timer m_timer = new Timer();

  //SET MOTOR OUTPUT METHODS
  private void setSpeed(double intakeValue, double holderValue){
    m_holder.set(holderValue);
    setShooterSpeed(intakeValue);
  }

  private void setShooterSpeed(double intakeValue){
    m_toasterRight.set(intakeValue);
    m_toasterLeft.set(intakeValue);
  }

  private double getRPM(){
    return m_toasterEncoder.getVelocity();
  }

  private boolean atShootingSpeed(){
    return (getRPM() >= ToasterConstants.kShootingSpeed);
  }

  @Override
  public void periodic() {
    logData();
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

  public Command ToasterOffCmd() {
    return this.runOnce(() -> {
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
    .withTimeout(ToasterConstants.SpeakerWaitTime)
    .andThen(this.run(() -> m_holder.set(ToasterConstants.SpeakerHolderSpeed)));
  }

  public Command ToasterAmpShootCmd() {
    return this.run(() -> setSpeed(ToasterConstants.AmpShooterSpeed, ToasterConstants.AmpHolderSpeed));
  }

}