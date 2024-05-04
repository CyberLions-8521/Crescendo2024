package frc.robot.Util;
// import static frc.robot.Constants.SwerveModuleConstants.CIRCUMFERENCE;
// import static frc.robot.Constants.SwerveModuleConstants.TURN_GEAR_RATIO;
// import static frc.robot.Constants.SwerveModuleConstants.TURN_KP;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.SwerveModuleConstants.*;

public class SwerveModule {
     private TalonFX m_driveMotor;
     private CANSparkMax m_turnMotor;
     private SparkPIDController m_turnController;
     private RelativeEncoder m_turnEncoder;
     private CANcoder m_canCoder;
     private VelocityDutyCycle targetSpeed = new VelocityDutyCycle(0);
     public Orchestra m_Orchestra = new Orchestra();

     public SwerveModule(int drivePort, int turnPort, int encoderPort) {
          Timer.delay(0.1);

          m_driveMotor = new TalonFX(drivePort, "Ryan");
          m_turnMotor  = new CANSparkMax(turnPort, MotorType.kBrushless);
          m_canCoder  = new CANcoder(encoderPort, "rio");
          m_turnController = m_turnMotor.getPIDController();
          m_turnEncoder = m_turnMotor.getEncoder();

          m_Orchestra.addInstrument(m_driveMotor);
     }

     public void applyConfigs(final TalonFXConfiguration krakenCfg, MagnetSensorConfigs canCoderCfg) {
          m_driveMotor.getConfigurator().apply(krakenCfg);
          m_driveMotor.setPosition(0);
          m_canCoder.getConfigurator().apply(canCoderCfg);
          configTurnMotor(true);
          rezeroTurnMotors();
     }

     private void configTurnMotor(final boolean isInverted) {
          m_turnMotor.restoreFactoryDefaults();
          m_turnEncoder.setPositionConversionFactor(1/TURN_GEAR_RATIO * 2 * Math.PI);
          m_turnEncoder.setVelocityConversionFactor(1/TURN_GEAR_RATIO * 2 * Math.PI / 60);
          m_turnController.setP(TURN_KP);
          m_turnController.setPositionPIDWrappingEnabled(true);
          m_turnController.setPositionPIDWrappingMinInput(0);
          m_turnController.setPositionPIDWrappingMaxInput(2 * Math.PI);
          m_turnMotor.setIdleMode(IdleMode.kCoast);
          m_turnMotor.setInverted(isInverted);
          m_turnMotor.setSmartCurrentLimit(20,20);
          m_turnMotor.burnFlash();
     }

     public void rezeroTurnMotors() {
          //REZERO TURN MOTORS
          //absolute rotaion of the cancoder * mt/r
          m_turnEncoder.setPosition(getAbsoluteTurnAngle().getRadians());
     }

     public Rotation2d getAbsoluteTurnAngle() {
          // angleGetter.refresh();
          //getabsposition returns status signal of type double / rotations
          //get value takes the type value and returns it.
          return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValue());
     }

     public SwerveModuleState getState(){
          return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), getTurnAngle());
     }
     
     public void setState(SwerveModuleState state) {
          SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
          // optimizedState.speedMetersPerSecond *= state.angle.minus(getTurnAngle()).getCos();
          m_driveMotor.setControl(targetSpeed.withVelocity(optimizedState.speedMetersPerSecond));
          m_turnController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
     }

     public Rotation2d getTurnAngle() {
          return new Rotation2d(m_turnEncoder.getPosition());
     }
     public double getDrivePosition() {
         //return (m_driveMotor.getPosition().getValueAsDouble()/Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO) * CIRCUMFERENCE;
         return m_driveMotor.getPosition().getValueAsDouble();
     }
     public double getDriveVelocity() {
          //return (m_driveMotor.getVelocity().getValueAsDouble()/Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO) * CIRCUMFERENCE;
          return m_driveMotor.getVelocity().getValueAsDouble();
     }

     public SwerveModulePosition getModulePosition(){
          //mt / gear
          //m / mt/r
          //m (r/mt)
          return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
     }
}