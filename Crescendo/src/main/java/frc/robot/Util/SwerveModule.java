package frc.robot.Util;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
     CANSparkMax m_driveMotor;
     CANSparkMax m_turnMotor;

     SparkPIDController m_driveController;
     SparkPIDController m_turnController;

     RelativeEncoder m_turnEncoder;
     RelativeEncoder m_driveEncoder;

     private CANcoder m_canCoder;

     public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
          m_driveMotor = new CANSparkMax(0,MotorType.kBrushless);
          m_turnMotor  = new CANSparkMax(turnPort, MotorType.kBrushless);
          m_canCoder   = new CANcoder(encoderPort);

          m_driveController = m_driveMotor.getPIDController();
          m_turnController  = m_turnMotor.getPIDController();
          configMotors(isInverted);
          configCANcoder(angleOffset);
          zeroEncoders();
     }

     public void configMotors(boolean isInverted){
          m_driveMotor.restoreFactoryDefaults();
          m_turnMotor.restoreFactoryDefaults();

          m_driveMotor.setIdleMode(IdleMode.kBrake);
          m_turnMotor.setIdleMode(m_driveMotor.getIdleMode());

          m_driveMotor.setInverted(isInverted);
          m_driveMotor.setInverted(false);

          m_driveMotor.setSmartCurrentLimit(15,15);
          m_turnMotor.setSmartCurrentLimit(15,15);

          m_driveController.setP(SwerveModuleConstants.DRIVE_KP);
          m_driveController.setFF(SwerveModuleConstants.DRIVE_KFF);

          m_turnController.setP(SwerveModuleConstants.TURN_KP);
     }

     public void configCANcoder(double angleOffset){
          CANcoderConfiguration m_config = new CANcoderConfiguration();
          m_config.MagnetSensor.AbsoluteSensorRange = m_config.MagnetSensor.AbsoluteSensorRange.Signed_PlusMinusHalf;
          m_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
          //change initlization strategry
          m_config.MagnetSensor.MagnetOffset = angleOffset;
     }

     public void configGains(){
          m_turnController.setP(SmartDashboard.getNumber("Turn P", 0));
          m_driveController.setP(SmartDashboard.getNumber("Drive P", 0));
          m_driveController.setFF(SmartDashboard.getNumber("Drive FF", 0));
     }

     public void zeroEncoders(){
          m_turnEncoder.setPosition(0);
          m_driveEncoder.setPosition(0);
     }

     public void rezeroTurnMotors(){
          m_turnEncoder.setPosition((m_canCoder.getAbsolutePosition().getValueAsDouble() / 360) * SwerveModuleConstants.TURN_GEAR_RATIO);
     }

     public void setTurnDegrees(Rotation2d turnSetpoint){
          //Rotations * Gear Ratio
          //Rotations * (Motor Revolutions / 1 Rotation)
          //Motor Revolutions
          m_turnController.setReference(turnSetpoint.getRotations() * SwerveModuleConstants.TURN_GEAR_RATIO, ControlType.kPosition);
     }

     public void setDriveVelocity(double metersPerSec){
          if(metersPerSec == 0){
               m_driveMotor.set(0);
          }
          
          double RPM = (metersPerSec * 60) / SwerveModuleConstants.CIRCUMFERENCE;
          m_driveController.setReference(RPM, ControlType.kVelocity);
     }

     public void setState(SwerveModuleState state){
          SwerveModuleState optimizedState = CTREUtils.optimize(state, getTurnAngle());
     }

     public Rotation2d getTurnAngle(){
          //mt / (mt / rotation) -- > mt (r/mt) -- > rotations
          return Rotation2d.fromRotations(m_turnEncoder.getPosition() / SwerveModuleConstants.TURN_GEAR_RATIO); 
     }
     
     public Rotation2d getAbsoluteTurnAngle(){
          //getabsposition returns status signal of type double / rotations
          //get value takes the type value and returns it.
          return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValue());
     }

     public double getDriveVelocity(){
          //RPM --> m/s
          //RPM / 60 = Rotations per second
          //rotations per sec * gear ratio
          //rotations per second * (motor turns / 1 rotation)
          //motor turns per second
          //motor turns per second * circumference
          double RPS = m_driveEncoder.getVelocity() * 60;
          double rotations = RPS * SwerveModuleConstants.DRIVE_GEAR_RATIO;
          return (rotations * SwerveModuleConstants.DRIVE_GEAR_RATIO);
     }

     public double getDrivePosition(){
          //motor turns / gear ratio
          //motor turns / (motor turns / 1 revolution)
          //1 revolution * circumference
          return ((m_driveEncoder.getPosition() / SwerveModuleConstants.DRIVE_GEAR_RATIO) * SwerveModuleConstants.CIRCUMFERENCE);
     }


}
