package frc.robot.Util;
// import static frc.robot.Constants.SwerveModuleConstants.CIRCUMFERENCE;
// import static frc.robot.Constants.SwerveModuleConstants.TURN_GEAR_RATIO;
// import static frc.robot.Constants.SwerveModuleConstants.TURN_KP;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.SwerveModuleConstants.*;

public class SwerveModule {
     //MOTOR OBJECTS
     TalonFX m_driveMotor;
     CANSparkMax m_turnMotor;
     
     //PID CONTROLLERS
     //Drive PID Controller configured in configMotors();
     SparkPIDController m_turnController;

     //ENCODER OBJECTS
     public RelativeEncoder m_turnEncoder;

     //CANCODER OBJECT 
     private CANcoder m_canCoder;

     //PID CONTROLLER --> DRIVE MOTOR TARGET SPEED OBJECT
     private VelocityDutyCycle targetSpeed = new VelocityDutyCycle(0);

     //MUSIC GANGSTA RAP
     public Orchestra m_Orchestra = new Orchestra();


     public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
          Timer.delay(0.1);

          //CREATE MOTORS
          m_driveMotor = new TalonFX(drivePort, "Ryan");
          m_turnMotor  = new CANSparkMax(turnPort, MotorType.kBrushless);

          //CREATE CANCODER
          m_canCoder  = new CANcoder(encoderPort, "rio");

          //CREATE PID CONTROLLER
          m_turnController = m_turnMotor.getPIDController();
    
          //CREATE TURN ENCODER
          m_turnEncoder = m_turnMotor.getEncoder();

          //CONFIGURATIONS
          configMotors(isInverted);
          configCANcoder(angleOffset);
          rezeroTurnMotors();
          zeroEncoders();

          m_Orchestra.addInstrument(m_driveMotor);
     }

     public void configCANcoder(double angleOffset){
          //CONFIGURE CANCODER
          var m_config = new CANcoderConfiguration();
          m_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
          m_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
          m_config.MagnetSensor.MagnetOffset = angleOffset;

          m_canCoder.getConfigurator().apply(m_config);
     }

     public void configGains(){
          //CONFIGURE TURN CONTROLLER GAINS
          m_turnController.setP(SmartDashboard.getNumber("Turn P", 0));
          
          Slot0Configs m_slotConfig = new Slot0Configs();
          
          //CONFIGURE DRIVE CONTROLLER GAINS
          m_slotConfig.kP = SmartDashboard.getNumber("Drive P", 0);
          m_slotConfig.kD = SmartDashboard.getNumber("Drive D", 0);
          m_slotConfig.kV = SmartDashboard.getNumber("Drive FF", 0);
          m_driveMotor.getConfigurator().apply(m_slotConfig);
     }

     public void playMusic(String pathname){
          m_Orchestra.loadMusic(pathname);
          m_Orchestra.play();

     }



     public void zeroEncoders(){
          m_driveMotor.setPosition(0);
     }

     public void rezeroTurnMotors(){
          //REZERO TURN MOTORS
          //absolute rotaion of the cancoder * mt/r
          m_turnEncoder.setPosition(getAbsoluteTurnAngle().getRotations() * TURN_GEAR_RATIO);
     }

     public void setTurnDegrees(SwerveModuleState desiredTurn){
          //Rotations * Gear Ratio
          //Rotations * (Motor Revolutions / 1 Rotation)
          //Motor Revolutions 
          
          m_turnController.setReference(desiredTurn.angle.getRotations()*TURN_GEAR_RATIO, ControlType.kPosition);
          SmartDashboard.putNumber("measured", m_turnEncoder.getPosition());
          SmartDashboard.putNumber("setpoint", desiredTurn.angle.getRotations());
     }
     public void setDriveVelocity(double metersPerSec){
          if(metersPerSec == 0){
               m_driveMotor.set(0);
          }else{
               //double RPS = (metersPerSec / SwerveModuleConstants.CIRCUMFERENCE) * SwerveModuleConstants.DRIVE_GEAR_RATIO;
               //m_driveMotor.setControl(targetSpeed.withVelocity(RPS));
               m_driveMotor.setControl(targetSpeed.withVelocity(metersPerSec).withEnableFOC(true));
          }

     }

     public void setState(SwerveModuleState state){
          SwerveModuleState optimizedState = CTREUtils.optimize(state, getTurnAngle());
          setDriveVelocity(optimizedState.speedMetersPerSecond);
          setTurnDegrees(optimizedState);
     }

     public SwerveModuleState getState(){
          return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), getTurnAngle());
     }

     public Rotation2d getTurnAngle(){
          return Rotation2d.fromRotations(m_turnEncoder.getPosition() / TURN_GEAR_RATIO);
     }
     public double getDrivePosition(){
         //return (m_driveMotor.getPosition().getValueAsDouble()/Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO) * CIRCUMFERENCE;
         return m_driveMotor.getPosition().getValueAsDouble();
     }
     public double getDriveVelocity(){
          //return (m_driveMotor.getVelocity().getValueAsDouble()/Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO) * CIRCUMFERENCE;
          return m_driveMotor.getVelocity().getValueAsDouble();
     }
     
     public Rotation2d getAbsoluteTurnAngle(){
          // angleGetter.refresh();
          //getabsposition returns status signal of type double / rotations
          //get value takes the type value and returns it.
          return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValue());
     }


     public SwerveModulePosition getModulePosition(){
          //mt / gear
          //m / mt/r
          //m (r/mt)
          return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
     }

     public void configMotors(boolean isInverted){
          //TURN MOTOR
          m_turnMotor.restoreFactoryDefaults();
          m_turnMotor.setIdleMode(IdleMode.kCoast);
          m_turnMotor.setInverted(true);

          m_turnController.setP(TURN_KP);

          //NEW LINE
          m_turnMotor.setSmartCurrentLimit(40,40);

          m_turnMotor.burnFlash();
          //DRIVE MOTOR
          TalonFXConfiguration m_driveControllerConfig = new TalonFXConfiguration();
          
          //CONFIGURE PID VALUES
          m_driveControllerConfig.Slot0.kP = DRIVE_KP;
          m_driveControllerConfig.Slot0.kD = DRIVE_KD;
          m_driveControllerConfig.Slot0.kV = DRIVE_KFF;

          //NEW LINE
          m_driveControllerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
          m_driveControllerConfig.CurrentLimits.SupplyCurrentLimit = 80;

          //INVERSION
          m_driveControllerConfig.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

          m_driveControllerConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO / CIRCUMFERENCE;

          //NEUTRAL MODE
          m_driveControllerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

          //APPLY IT
          m_driveMotor.getConfigurator().apply(m_driveControllerConfig);

          zeroEncoders();
     }
}