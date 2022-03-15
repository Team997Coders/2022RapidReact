// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDS. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  public LEDs() {
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED_COUNT);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
