package frc.robot.Util;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
     private StatusSignal<Double> angleGetter;// = m_canCoder.getAbsolutePosition();

     public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
          m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
          m_turnMotor  = new CANSparkMax(turnPort, MotorType.kBrushless);
          m_canCoder   = new CANcoder(encoderPort);
          angleGetter = m_canCoder.getAbsolutePosition();



          m_driveController = m_driveMotor.getPIDController();
          m_turnController  = m_turnMotor.getPIDController();

          m_driveEncoder = m_driveMotor.getEncoder();
          m_turnEncoder = m_turnMotor.getEncoder();

          configMotors(isInverted);
          configCANcoder(angleOffset);
          zeroEncoders();
          rezeroTurnMotors();
     }

     public void configCANcoder(double angleOffset){
          CANcoderConfiguration m_config = new CANcoderConfiguration();
          m_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
          m_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
          //change initlization strategry
          m_config.MagnetSensor.MagnetOffset = angleOffset / 360;

          m_canCoder.getConfigurator().apply(m_config);
     }

     public void configGains(){
          m_turnController.setP(SmartDashboard.getNumber("Turn P", SwerveModuleConstants.TURN_KP));
          m_driveController.setP(SmartDashboard.getNumber("Drive P", SwerveModuleConstants.DRIVE_KP));
          m_driveController.setD(SmartDashboard.getNumber("Drive D", SwerveModuleConstants.DRIVE_KD));
          m_driveController.setFF(SmartDashboard.getNumber("Drive FF", SwerveModuleConstants.DRIVE_KFF));
     }

     public void zeroEncoders(){
          m_turnEncoder.setPosition(0);
          m_driveEncoder.setPosition(0);
     }

     public void rezeroTurnMotors(){
          m_turnEncoder.setPosition(-getAbsoluteTurnAngle().getRotations() * SwerveModuleConstants.TURN_GEAR_RATIO);
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
          else{
               double RPM = ((metersPerSec * 60) / SwerveModuleConstants.CIRCUMFERENCE) * SwerveModuleConstants.DRIVE_GEAR_RATIO;
               m_driveController.setReference(RPM, ControlType.kVelocity);
          }
     }

     public void setState(SwerveModuleState state){
          SwerveModuleState optimizedState = CTREUtils.optimize(state, getTurnAngle());// state.optimize(state, getTurnAngle());
          setDriveVelocity(optimizedState.speedMetersPerSecond);
          setTurnDegrees(optimizedState.angle);
     }

     public SwerveModuleState getState(){
          return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
     }

     public Rotation2d getTurnAngle(){
          //mt / (mt / rotation) -- > mt (r/mt) -- > rotations
          return Rotation2d.fromRotations(m_turnEncoder.getPosition() / SwerveModuleConstants.TURN_GEAR_RATIO); 
     }
     
     public Rotation2d getAbsoluteTurnAngle(){
          angleGetter.refresh();
          //getabsposition returns status signal of type double / rotations
          //get value takes the type value and returns it.
          return Rotation2d.fromRotations(angleGetter.getValue());
     }
     
     public double getDriveVelocity(){
          //RPM --> m/s
          //RPM / 60 = Rotations per second
          //rotations per sec * gear ratio
          //rotations per second * (motor turns / 1 rotation)
          //motor turns per second
          //motor turns per second * circumference
          double motorRPS = m_driveEncoder.getVelocity() / 60;
          double wheelRPS = motorRPS / SwerveModuleConstants.DRIVE_GEAR_RATIO;
          return (wheelRPS * SwerveModuleConstants.CIRCUMFERENCE);
     }

     public double getDrivePosition(){
          //motor turns / gear ratio
          //motor turns / (motor turns / 1 revolution)
          //1 revolution * circumference
          return ((m_driveEncoder.getPosition() / SwerveModuleConstants.DRIVE_GEAR_RATIO) * SwerveModuleConstants.CIRCUMFERENCE);
     }

     public SwerveModulePosition getModulePosition(){
          return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
     }

     public void configMotors(boolean isInverted){
          m_driveMotor.restoreFactoryDefaults();
          m_turnMotor.restoreFactoryDefaults();

          m_driveMotor.setIdleMode(IdleMode.kBrake);
          m_turnMotor.setIdleMode(m_driveMotor.getIdleMode());

          m_driveMotor.setInverted(isInverted);

          m_driveMotor.setSmartCurrentLimit(15,15);
          m_turnMotor.setSmartCurrentLimit(15,15);

          m_turnMotor.setInverted(false);

          m_driveController.setP(SwerveModuleConstants.DRIVE_KP);
          m_driveController.setD(SwerveModuleConstants.DRIVE_KD);
          m_driveController.setFF(SwerveModuleConstants.DRIVE_KFF);

          m_turnController.setP(SwerveModuleConstants.TURN_KP);
     }
}
