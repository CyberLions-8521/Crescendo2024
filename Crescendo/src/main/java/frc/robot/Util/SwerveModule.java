package frc.robot.Util;
import static frc.robot.Constants.SwerveModuleConstants.TURN_GEAR_RATIO;
import static frc.robot.Constants.SwerveModuleConstants.kDriveKinematics;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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


     //CANCODER OBJECT 
     private CANcoder m_canCoder;
     boolean lol = false;

     // private PIDController m_turnPID = new PIDController(0.01, 0, 0);

     
     //PID CONTROLLER --> DRIVE MOTOR TARGET SPEED OBJECT
     private VelocityDutyCycle targetSpeed = new VelocityDutyCycle(0);
     // private VelocityVoltage targetVoltage;
     // private VoltageOut targetVoltageOut;

     public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
          var m_driveControllerConfig = new TalonFXConfiguration();
          

          //CONFIGURE PID VALUES
          m_driveControllerConfig.Slot0.kP = SwerveModuleConstants.DRIVE_KP;
          m_driveControllerConfig.Slot0.kD = SwerveModuleConstants.DRIVE_KD;
          m_driveControllerConfig.Slot0.kV = SwerveModuleConstants.DRIVE_KFF;


          
          //CREATE MOTORS
          m_driveMotor = new TalonFX(drivePort, "Ryan");
          m_turnMotor  = new CANSparkMax(turnPort, MotorType.kBrushless);

          m_turnMotor.restoreFactoryDefaults();

          m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
          m_turnMotor.setIdleMode(IdleMode.kBrake);
          m_turnMotor.burnFlash();

          //CREATE CANCODER
          m_canCoder  = new CANcoder(encoderPort, "rio");
          // angleGetter = m_canCoder.getAbsolutePosition();

          //CREATE PID CONTROLLER
          m_turnController = m_turnMotor.getPIDController();

          // targetVoltage = new VelocityVoltage(0).withSlot(0);
          // targetVoltageOut = new VoltageOut(0);
          
          //CREATE TURN ENCODER
          m_turnEncoder = m_turnMotor.getEncoder();

          //var slot_amongus = new FeedbackConfigs();
          //slot_amongus.SensorToMechanismRatio = Constants.SwerveModuleConstants.CIRCUMFERENCE/Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO;

          //m_driveMotor.getConfigurator().apply(slot_amongus);
          var slot_0Output = new MotorOutputConfigs();
          // System.out.println("Inverted");
          // lol = true;
          // m_driveMotor.setInverted(lol);
          if(drivePort == 3){
               slot_0Output.Inverted = InvertedValue.CounterClockwise_Positive;
          }else{
               slot_0Output.Inverted = InvertedValue.Clockwise_Positive;
          }
          
          // m_driveMotor.getConfigurator().refresh(slot_0Output);
          m_driveMotor.getConfigurator().apply(slot_0Output);

          // m_turnEncoder.setPositionConversionFactor(1/Constants.SwerveModuleConstants.TURN_GEAR_RATIO);


          // m_turnController.setP(Constants.SwerveModuleConstants.TURN_KP);
          //CONFIGURATIONS
          // configGains();
          // configMotors(false);
          configCANcoder(-angleOffset);
          // resetToAbsolute();
          rezeroTurnMotors();
          zeroEncoders();
         
     }

     public void configCANcoder(double angleOffset){
          //CONFIGURE CANCODER
          var m_config = new CANcoderConfiguration();
          m_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
          m_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
          m_config.MagnetSensor.MagnetOffset = angleOffset;
          //m_config.MagnetSensor.MagnetOffset = angleOffset / 180;

          m_canCoder.getConfigurator().apply(m_config);
     }



     public void configGains(){
          //CONFIGURE TURN CONTROLLER GAINS
          m_turnController.setP(SmartDashboard.getNumber("Turn P", 0));
          
          var m_driveControllerConfig = new TalonFXConfiguration();
          
          //CONFIGURE DRIVE CONTROLLER GAINS
          m_driveControllerConfig.Slot0.kP = SwerveModuleConstants.DRIVE_KP;
          m_driveControllerConfig.Slot0.kD = SwerveModuleConstants.DRIVE_KD;
          m_driveControllerConfig.Slot0.kV = SwerveModuleConstants.DRIVE_KFF;
          m_driveMotor.getConfigurator().apply(m_driveControllerConfig);
     }

     public void zeroEncoders(){
          //ZERO ENCODERS
          m_driveMotor.setPosition(0);
     }

     public void setAngle(SwerveModuleState desiredState){
          m_turnController.setReference(desiredState.angle.getRotations()*TURN_GEAR_RATIO, ControlType.kPosition);
     }

     public Rotation2d getCanCoder(){
          return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble());
     }
     //
     public void resetToAbsolute(){
          double absolutePosition = getCanCoder().getRotations();
          m_turnEncoder.setPosition(absolutePosition);
     }

     public void rezeroTurnMotors(){
          //REZERO TURN MOTORS
          //absolute rotaion of the cancoder * mt/r
          m_turnEncoder.setPosition(getAbsoluteTurnAngle().getRotations() * SwerveModuleConstants.TURN_GEAR_RATIO);
          // m_turnEncoder.setPosition(m_canCoder.getAbsolutePosition() / (360) * (SwerveModuleConstants.TURN_GEAR_RATIO));
          // m_turnEncoder.setFeedbackDevice(m_canCoder);
     }

     public void setTurnDegrees(SwerveModuleState desiredTurn){
          //Rotations * Gear Ratio
          //Rotations * (Motor Revolutions / 1 Rotation)
          //Motor Revolutions 
          
          m_turnController.setReference(desiredTurn.angle.getRotations()*TURN_GEAR_RATIO, ControlType.kPosition);
          // m_turnMotor.set(0);
          // m_turnMotor.set(m_turnPID.calculate(m_turnEncoder.getPosition(),desiredTurn.angle.getRotations()));
          SmartDashboard.putNumber("measured", m_turnEncoder.getPosition());
          SmartDashboard.putNumber("setpoint", desiredTurn.angle.getRotations());
          // m_turnController.setReference(turnSetpoint.getRotations() * SwerveModuleConstants.TURN_GEAR_RATIO, ControlType.kPosition);
     }
     public void setDriveVelocity(double metersPerSec){
          if(metersPerSec == 0){
               m_driveMotor.set(0);
          }else{
               double RPS = (metersPerSec / SwerveModuleConstants.CIRCUMFERENCE) * SwerveModuleConstants.DRIVE_GEAR_RATIO;
               m_driveMotor.setControl(targetSpeed.withVelocity(RPS));
          }
          // double RPM = ((metersPerSec * 60) / SwerveModuleConstants.CIRCUMFERENCE) * SwerveModuleConstants.DRIVE_GEAR_RATIO;
          // m_driveController.setReference(RPM, ControlType.kVelocity);
          //setControl uses RPS
          
          // m_driveMotor.setControl(targetVoltage.withVelocity(RPS));
          // m_driveMotor.setControl(targetVoltageOut.withOutput(0.3));
     }

     public void setState(SwerveModuleState state){
          SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());

          // m_driveMotor.setControl(targetSpeed.withVelocity(0));
          
          // setAngle(optimizedState);
          // m_turnMotor.set(m_turnPID.calculate(0, 0));
          // m_turnMotor.set(m_turnPID.calculate(m_canCoder.getAbsolutePosition().getValueAsDouble(),optimizedState.angle.getRotations() ));
          // SwerveModuleState.optimize(state, getTurnAngle());
          setDriveVelocity(optimizedState.speedMetersPerSecond);
          setTurnDegrees(optimizedState);
          // m_turnMotor.set(0.2);
     }

     public SwerveModuleState getState(){
          return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), getTurnAngle());
     }

     public Rotation2d getTurnAngle(){
                    //mt / (mt / rotation) -- > mt (r/mt) -- > rotations
         // return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble());
          return Rotation2d.fromRotations(m_turnEncoder.getPosition() / Constants.SwerveModuleConstants.TURN_GEAR_RATIO);
          // return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValue() / SwerveModuleConstants.TURN_GEAR_RATIO); 
     }
     public double getDrivePosition(){
          return m_driveMotor.getPosition().getValueAsDouble()/Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO;
     }
     
     public Rotation2d getAbsoluteTurnAngle(){
          // angleGetter.refresh();
          //getabsposition returns status signal of type double / rotations
          //get value takes the type value and returns it.
          // return Rotation2d.fromRotations(angleGetter.getValue());
          return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValue());
     }


     public SwerveModulePosition getModulePosition(){
          //mt / gear
          //m / mt/r
          //m (r/mt)
          return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
     }

     public void configMotors(boolean isInverted){
          
          // m_turnController.setP(0.04);

          //CONFIGURE CURRENT LIMITS
          // CurrentLimitsConfigs m_driveCurrentLimits = new CurrentLimitsConfigs();
          // m_driveCurrentLimits.StatorCurrentLimit = 40;
          // m_driveCurrentLimits.StatorCurrentLimitEnable = true;   

          //CONFIGURE IDLE MODE
          
          m_turnMotor.setIdleMode(IdleMode.kBrake);

          //CONFIGURE INVERSION
          // m_turnMotor.setInverted(isInverted);


          //APPLY CURRENT LIMITS
          //m_driveControllerConfig.CurrentLimits = m_driveCurrentLimits;
          //m_driveMotor.getConfigurator().apply(m_driveControllerConfig);
          //m_turnMotor.setSmartCurrentLimit(40,40);  

          //RESTORE FACTORY DEFAULT
          zeroEncoders();
     }
}