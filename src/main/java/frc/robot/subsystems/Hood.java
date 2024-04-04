// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorConstants;
// import frc.robot.subsystems.Elevator.ElevatorState;

public class Hood extends SubsystemBase {

  //CONSTRUCTOR
  public Hood() {
    configMotors();
  }
 
  //STATES
  // public enum HoodState{
  //   OFF,
  //   ON
  // }

  //INSTANCE
  // private static Hood m_instance = new Hood();

  //SET STATE TO OFF
  // private HoodState m_state = HoodState.OFF;

  //MOTOR OBJECT
  private CANSparkMax m_hoodMaster = new CANSparkMax(MotorConstants.HOOD_MOTOR, MotorType.kBrushless);

  //GET INSTANCE
  // public static Hood getInstance(){
  //   return m_instance;
  // }  

  //STATE METHODS
  // public void setState(HoodState m_state){
  //   this.m_state = m_state;
  // }
  
  // public HoodState getState(){
  //   return m_state;
  // }

  //SET MOTOR OUTPUT
  public void setSpeed(double value){
    m_hoodMaster.set(value);
  }

  @Override
  public void periodic() {
    // switch(m_state){
    //   case OFF:
    //     break;
    //   case ON:
    //     setSpeed(HoodConstants.HOOD_SPEED);
    //     break;
    // }   
    logData();
  }

  public Command HoodSetSpeedCmd(final double speed) {
    return this.run(() -> setSpeed(speed));
  }

  public void logData(){
    // SmartDashboard.putString("hood State", getState().toString());
    SmartDashboard.putNumber("hood motor output", m_hoodMaster.get());
  }

  public void configMotors(){
    m_hoodMaster.restoreFactoryDefaults();
    m_hoodMaster.setInverted(false);
    m_hoodMaster.setIdleMode(IdleMode.kCoast);
    m_hoodMaster.burnFlash();
  }
}