package frc.robot.Util;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

public class SwerveModule {
     //MOTOR OBJECTS
     TalonFX m_driveMotor;
     CANSparkMax m_turnMotor;
     
     //PID CONTROLLERS
     //Drive PID Controller configured in configMotors();
     SparkPIDController m_turnController;

     //ENCODER OBJECTS
     RelativeEncoder m_turnEncoder;

     //CREATE CONFIGURATION OBJECT
     TalonFXConfiguration m_driveControllerConfig;

     //CANCODER OBJECT 
     private CANcoder m_canCoder;
     private StatusSignal<Double> angleGetter;

     
     //PID CONTROLLER --> DRIVE MOTOR TARGET SPEED OBJECT
     private VelocityDutyCycle targetSpeed;
     // private VelocityVoltage targetVoltage;
     // private VoltageOut targetVoltageOut;

     public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
          //CREATE MOTORS
          m_driveMotor = new TalonFX(drivePort, "Ryan");
          m_turnMotor  = new CANSparkMax(turnPort, MotorType.kBrushless);

          //CREATE CANCODER
          m_canCoder   = new CANcoder(encoderPort, "rio");
          angleGetter = m_canCoder.getAbsolutePosition();

          //CREATE PID CONTROLLER
          m_turnController  = m_turnMotor.getPIDController();

          targetSpeed = new VelocityDutyCycle(0).withSlot(0);
          // targetVoltage = new VelocityVoltage(0).withSlot(0);
          // targetVoltageOut = new VoltageOut(0);
          
          //CREATE TURN ENCODER
          m_turnEncoder = m_turnMotor.getEncoder();

          //CONFIGURATIONS
          configMotors(isInverted);
          configCANcoder(angleOffset);
          zeroEncoders();
          rezeroTurnMotors();

     }

     public void configCANcoder(double angleOffset){
          //CONFIGURE CANCODER
          //missing restore factory defaualts
          CANcoderConfiguration m_config = new CANcoderConfiguration();
          m_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
          m_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
          // m_config.MagnetSensor.MagnetOffset = angleOffset / 360;
          m_config.MagnetSensor.MagnetOffset = angleOffset / 180;
          //change initlization strategry (automatically does it)

          m_canCoder.getConfigurator().apply(m_config);
     }

     public void configGains(){
          //CONFIGURE TURN CONTROLLER GAINS
          m_turnController.setP(SmartDashboard.getNumber("Turn P", 0));

          //CONFIGURE DRIVE CONTROLLER GAINS
          m_driveControllerConfig.Slot0.kP = (SmartDashboard.getNumber("Drive P", 0));
          m_driveControllerConfig.Slot0.kD = (SmartDashboard.getNumber("Drive D", 0));
          m_driveControllerConfig.Slot0.kV = (SmartDashboard.getNumber("Drive FF", 0));
          m_driveMotor.getConfigurator().apply(m_driveControllerConfig);
     }

     public void zeroEncoders(){
          //ZERO ENCODERS
          m_turnEncoder.setPosition(0);
          m_driveMotor.setPosition(0);
     }

     public void rezeroTurnMotors(){
          //REZERO TURN MOTORS
          //absolute rotaion of the cancoder * mt/r
          m_turnEncoder.setPosition(-getAbsoluteTurnAngle().getRotations() * SwerveModuleConstants.TURN_GEAR_RATIO);
          // m_turnEncoder.setFeedbackDevice(m_canCoder);
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
               // double RPM = ((metersPerSec * 60) / SwerveModuleConstants.CIRCUMFERENCE) * SwerveModuleConstants.DRIVE_GEAR_RATIO;
               // m_driveController.setReference(RPM, ControlType.kVelocity);
               //setControl uses RPS
               double RPS = (metersPerSec / SwerveModuleConstants.CIRCUMFERENCE) * SwerveModuleConstants.DRIVE_GEAR_RATIO;
               m_driveMotor.setControl(targetSpeed.withVelocity(RPS));
               // m_driveMotor.setControl(targetVoltage.withVelocity(RPS));
               // m_driveMotor.setControl(targetVoltageOut.withOutput(0.3));

          }
     }

     public void setState(SwerveModuleState state){
          SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
          // CTREUtils.optimize(state, getTurnAngle());
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
          // angleGetter.refresh();
          //getabsposition returns status signal of type double / rotations
          //get value takes the type value and returns it.
          // return Rotation2d.fromRotations(angleGetter.getValue());
          return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValue());
     }
     
     public double getDriveVelocity(){
          //RPs --> m/s
          //rotations per sec * gear ratio
          //rotations per second * (motor turns / 1 rotation)
          //motor turns per second
          //motor turns per second * circumference
          double motorRPS = m_driveMotor.getVelocity().getValue();
          double wheelRPS = motorRPS / SwerveModuleConstants.DRIVE_GEAR_RATIO;
          return (wheelRPS * SwerveModuleConstants.CIRCUMFERENCE);
     }

     public double getDrivePosition(){
          //motor turns / gear ratio
          //motor turns / (motor turns / 1 revolution)
          //1 revolution * circumference
          return ((m_driveMotor.getPosition().getValue() / SwerveModuleConstants.DRIVE_GEAR_RATIO) * SwerveModuleConstants.CIRCUMFERENCE);
     }

     public SwerveModulePosition getModulePosition(){
          return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
     }

     public void configMotors(boolean isInverted){
          m_driveControllerConfig = new TalonFXConfiguration();

          //CONFIGURE PID VALUES
          m_driveControllerConfig.Slot0.kP = SwerveModuleConstants.DRIVE_KP;
          m_driveControllerConfig.Slot0.kD = SwerveModuleConstants.DRIVE_KD;
          m_driveControllerConfig.Slot0.kV = SwerveModuleConstants.DRIVE_KFF;

          m_turnController.setP(SwerveModuleConstants.TURN_KP);

          //CONFIGURE CURRENT LIMITS
          CurrentLimitsConfigs m_driveCurrentLimits = new CurrentLimitsConfigs();
          m_driveCurrentLimits.StatorCurrentLimit = 15;
          m_driveCurrentLimits.StatorCurrentLimitEnable = true;   

          //CONFIGURE IDLE MODE
          m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
          m_turnMotor.setIdleMode(IdleMode.kBrake);

          //CONFIGURE INVERSION
          m_driveMotor.setInverted(isInverted);
          m_turnMotor.setInverted(false);

          //APPLY CURRENT LIMITS
          m_driveControllerConfig.CurrentLimits = m_driveCurrentLimits;
          m_driveMotor.getConfigurator().apply(m_driveControllerConfig);
          m_turnMotor.setSmartCurrentLimit(15,15);  

          //RESTORE FACTORY DEFAULT
          m_turnMotor.restoreFactoryDefaults();
     }
}