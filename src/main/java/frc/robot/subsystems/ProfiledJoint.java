// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.JointConstants;

public class ProfiledJoint extends TrapezoidProfileSubsystem {
  //MOTOR OBJECTS
  private CANSparkMax m_jointRight = new CANSparkMax(MotorConstants.JOINT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_jointLeft = new CANSparkMax(MotorConstants.JOINT_LEFT_MOTOR, MotorType.kBrushless);

  //ENCODER OBJECT
  private RelativeEncoder m_jointEncoderRight = m_jointRight.getEncoder();
  private RelativeEncoder m_jointEncoderLeft = m_jointLeft.getEncoder();

  //MOTOR CONTROLLER OBJECT
  private SparkPIDController m_jointControllerLeft = m_jointLeft.getPIDController();

  /** Creates a new ProfiledJoint. */
  public ProfiledJoint() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(550, 150),
        // The initial position of the mechanism
        0);
    configMotors();
    rezero();
  }

  private void configMotors() {
    //RESTORE FACTORY DEFAULT
    m_jointRight.restoreFactoryDefaults();
    m_jointLeft.restoreFactoryDefaults();

    REVLibError checkOk;

    do {
      checkOk = m_jointRight.follow(m_jointLeft, true);
      Timer.delay(0.1);
    } while (checkOk != REVLibError.kOk);

    m_jointControllerLeft.setP(JointConstants.JOINT_KP);
    m_jointControllerLeft.setD(JointConstants.JOINT_KD);
    //IDLE MODE
    m_jointRight.setIdleMode(IdleMode.kBrake);
    m_jointLeft.setIdleMode(m_jointRight.getIdleMode());

    //SET SMART CURRENT LIMIT
    m_jointRight.setSmartCurrentLimit(40, 40);
    m_jointLeft.setSmartCurrentLimit(40, 40);

    m_jointEncoderRight.setMeasurementPeriod(12);
    m_jointEncoderRight.setAverageDepth(2);
    m_jointEncoderLeft.setMeasurementPeriod(12);
    m_jointEncoderLeft.setAverageDepth(2);
    
    m_jointRight.burnFlash();
    m_jointLeft.burnFlash();
  }

  public void rezero() {
    m_jointEncoderRight.setPosition(0);
    m_jointEncoderLeft.setPosition(0);
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    double output = MathUtil.clamp(state.position, 0, 33);
    m_jointControllerLeft.setReference(output, ControlType.kPosition);
  }

  //TRAPEZOID PROFILE OBJECT
  // private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(550, 150));
  // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  // private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  
  @Override
  public void periodic() {
    super.periodic();
    logData();  
  }

  private void logData() {
    SmartDashboard.putNumber("Joint kP", m_jointControllerLeft.getP());
    SmartDashboard.putNumber("Joint kd", m_jointControllerLeft.getD());
    // SmartDashboard.putNumber("Joint Goal Position", m_goal.position);
    // SmartDashboard.putNumber("Joint Goal Setpoint", m_setpoint.position);
    SmartDashboard.putNumber("joint velocity", m_jointEncoderLeft.getVelocity());
    SmartDashboard.putNumber("Joint Left", m_jointLeft.get());
    SmartDashboard.putNumber("Joint Right", m_jointRight.get());
    // SmartDashboard.putBoolean("At setpoint", isAtSetpoint());
    SmartDashboard.putNumber("Joint Position", m_jointEncoderLeft.getPosition());
  }

  public Command goToGoalCommand(final double goal) {
    return runOnce(() -> setGoal(goal));
  }
}
