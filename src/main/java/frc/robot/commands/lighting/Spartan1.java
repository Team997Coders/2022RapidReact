// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lighting;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class Spartan1 extends CommandBase {
  /** Creates a new Spartan1. */
  Lighting m_lighting;
  AddressableLEDBuffer m_buffer1, m_buffer2;
  long m_alternatingPeriodMS;
  long startTime;

  public Spartan1(Lighting lighting, long alternatingPeriodMS) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lighting);
    m_lighting = lighting;
    m_alternatingPeriodMS = alternatingPeriodMS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("init", "leds");
    m_buffer1 = new AddressableLEDBuffer(m_lighting.getLength());
    m_buffer2 = new AddressableLEDBuffer(m_lighting.getLength());
    for (int i = 0; i < m_buffer1.getLength(); i++) {
      if (i % 2 == 0) {
        m_buffer1.setRGB(i, 0, 0, 255);
        m_buffer2.setRGB(i, 255, 255, 255);
      }
      else {
        m_buffer2.setRGB(i, 0, 0, 255);
        m_buffer1.setRGB(i, 255, 255, 255);
      }
    }
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((System.currentTimeMillis() - startTime) % (m_alternatingPeriodMS * 2) < m_alternatingPeriodMS) {
      m_lighting.setLedBuffer(m_buffer1);
    }
    else {
      m_lighting.setLedBuffer(m_buffer2);
    }
    m_lighting.updateLeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
