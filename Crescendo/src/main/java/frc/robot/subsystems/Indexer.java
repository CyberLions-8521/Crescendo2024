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

public class Indexer extends SubsystemBase {

  //CONSTRUCTOR
  public Indexer() {
    configMotors();
  }
 
  //STATES
  public enum IndexerState{
    OFF,
    ON
  }

  private IndexerState m_state = IndexerState.OFF;

  private CANSparkMax m_indexerMaster = new CANSparkMax(MotorConstants.INDEXER_MOTOR, MotorType.kBrushless);

  public void setState(IndexerState m_state){
    this.m_state = m_state;
  }
  
  public IndexerState getState(){
    return m_state;
  }

  public void set(double value){
    m_indexerMaster.set(value);
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
    m_indexerMaster.restoreFactoryDefaults();
    m_indexerMaster.setInverted(false);
    m_indexerMaster.setIdleMode(IdleMode.kBrake);
    m_indexerMaster.setSmartCurrentLimit(40, 40);
  }
}