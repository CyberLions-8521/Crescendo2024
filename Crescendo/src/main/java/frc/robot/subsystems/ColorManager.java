// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;

public class ColorManager extends SubsystemBase {
  
  // takes the PWM port as an argument. It must be a PWM header on the roboRIO.
  private AddressableLED m_LED = new AddressableLED(ColorConstants.LED_PORT);
  //takes the number of LEDs as an input.
  private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(ColorConstants.BUFFER_LENGTH);

  public ColorManager() {

  }

  public enum ColorManagerState{
    OFF,
    RAINBOW,
    RED,
    YELLOW,
    GREEN,
    BLUE,
  }

  private ColorManagerState m_state = ColorManagerState.OFF;

  public void periodic() {
    logData();

    switch (m_state) {
      case OFF:
        break;
      case RAINBOW:
        break;
      case RED:
        break;
      case YELLOW:
        break;
      case GREEN:
        break;
      case BLUE:
        break;
    }
  }

  public void logData(){
    m_LED.setData(m_buffer);
    m_LED.start();
  }
}
