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
 
  //STATES
  public enum HoodState{
    OFF,
    ON
  }

  private HoodState m_state = HoodState.OFF;

  private CANSparkMax m_hoodMaster = new CANSparkMax(MotorConstants.HOOD_MOTOR, MotorType.kBrushless);

  public void setState(HoodState m_state){
    this.m_state = m_state;
  }
  
  public HoodState getState(){
    return m_state;
  }

  public void set(double value){
    m_hoodMaster.set(value);
  }

  @Override
  public void periodic() {
    switch(m_state){
      case OFF:
        set(0);
        break;
      case ON:
        set(0.8);
        break;
    }   
  }

  public void logData(){
    SmartDashboard.putString("Wrist State", getState().toString());
  }

  public void configMotors(){
    m_hoodMaster.restoreFactoryDefaults();
    m_hoodMaster.setInverted(false);
    m_hoodMaster.setIdleMode(IdleMode.kBrake);
    m_hoodMaster.setSmartCurrentLimit(40, 40);
  }
}