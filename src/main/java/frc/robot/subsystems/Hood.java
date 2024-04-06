// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorConstants;

public class Hood extends SubsystemBase {

  //CONSTRUCTOR
  public Hood() {
    configMotors();
  }

  //MOTOR OBJECT
  private CANSparkMax m_hoodMaster = new CANSparkMax(MotorConstants.HOOD_MOTOR, MotorType.kBrushless);

  //SET MOTOR OUTPUT
  private void setSpeed(double value){
    m_hoodMaster.set(value);
  }

  @Override
  public void periodic() {
    logData();
  }

  private void logData(){
    SmartDashboard.putNumber("hood motor output", m_hoodMaster.get());
  }

  private void configMotors(){
    m_hoodMaster.restoreFactoryDefaults();
    m_hoodMaster.setInverted(false);
    m_hoodMaster.setIdleMode(IdleMode.kCoast);
    m_hoodMaster.burnFlash();
  }

  public Command HoodSetSpeedCmd(final double speed) {
    return this.runOnce(() -> setSpeed(speed));
  }
}