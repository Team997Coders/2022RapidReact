// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lighting;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class AllianceColorsFlashing extends CommandBase {
  /** Creates a new AllianceColorsFlashing. */
  Lighting m_lighting;
  AddressableLEDBuffer m_buffer1, m_buffer2;
  double m_alternatingPeriodS;
  double startTime;

  /**
   * Sets the LEDs of the Lighting subsystem to flash the alliance colors, or if
   * not in match play flash blue.
   * 
   * @param lighting           : The {@link Lighting} subsystem to use.
   * @param alternatingPeriodS : The time of each flash (half the period of the
   *                           cycle).
   */
  public AllianceColorsFlashing(Lighting lighting, double alternatingPeriodS) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(lighting);
    m_lighting = lighting;
    m_alternatingPeriodS = alternatingPeriodS;
  }

  // Keeps LEDs active even when disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_buffer1 = new AddressableLEDBuffer(m_lighting.getLength());
    m_buffer2 = new AddressableLEDBuffer(m_lighting.getLength());
    for (int i = 0; i < m_buffer1.getLength(); i++) {
      if (i % 2 == 0) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          m_buffer1.setRGB(i, 255, 0, 0);
        } else {
          m_buffer1.setRGB(i, 0, 0, 255);
        }
        m_buffer2.setRGB(i, 255, 255, 255);
      } else {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          m_buffer2.setRGB(i, 255, 0, 0);
        } else {
          m_buffer2.setRGB(i, 0, 0, 255);
        }
        m_buffer1.setRGB(i, 255, 255, 255);
      }
    }
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((System.currentTimeMillis() - startTime) % (m_alternatingPeriodS * 2) < m_alternatingPeriodS) {
      m_lighting.setLedBuffer(m_buffer1);
    } else {
      m_lighting.setLedBuffer(m_buffer2);
    }
    m_lighting.updateLeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
