// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lighting extends SubsystemBase {
  /** Creates a new Lighting. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_length;

  public Lighting(int length) {
    m_length = length;
    m_led = new AddressableLED(Constants.Ports.LED_PWM_PORT);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);
  }

  public AddressableLEDBuffer getLedBuffer() { return m_ledBuffer; }
  public int getLength() { return m_length; }
  public void setLedBuffer(AddressableLEDBuffer buffer) 
  { 
    if (buffer.getLength() == m_length) {
      m_ledBuffer = buffer; 
    }
    SmartDashboard.putString("LED Status", "Invalid Buffer Length");
  }
  public void updateLeds() {
    if (m_ledBuffer == null) {
      m_ledBuffer = new AddressableLEDBuffer(m_length);
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
