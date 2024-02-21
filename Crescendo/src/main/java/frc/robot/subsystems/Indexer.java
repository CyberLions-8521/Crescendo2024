// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Indexer extends SubsystemBase {

  public Indexer() {
    configMotors();
  }

  private static Indexer m_instance = new Indexer();

  public static Indexer getInstance(){
    if (m_instance == null){
      m_instance = new Indexer();
    }
    return m_instance;
  }
 
  public enum IndexerState{
    OFF,
    STALL
  }

  private IndexerState m_state = IndexerState.OFF;

  private CANSparkMax m_indexerMaster = new CANSparkMax(MotorConstants.INDEXER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_indexerEncoder = m_indexerMaster.getEncoder();
  private SparkPIDController m_indexerController = m_indexerMaster.getPIDController();

  public void setState(IndexerState m_state){
    this.m_state = m_state;
  }
  
  public IndexerState getState(){
    return m_state;
  }

  public void set(double value){
    m_indexerMaster.set(value);
  }

  public void resetEncoder(){
    m_indexerEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    switch(m_state){
      case OFF:
        set(0);
        break;
      case STALL:
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