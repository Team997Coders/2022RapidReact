// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lighting;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class AllianceColors extends CommandBase {
  /** Creates a new AllianceColors. */
  Lighting m_lighting;
  AddressableLEDBuffer m_buffer;

  public AllianceColors(Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lighting);
    m_lighting = lighting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_buffer = new AddressableLEDBuffer(m_lighting.getLength());
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      for (int i = 0; i < m_buffer.getLength(); i++) {
        m_buffer.setRGB(i, 255, 0, 0);
      }
    } else {
      for (int i = 0; i < m_buffer.getLength(); i++) {
        m_buffer.setRGB(i, 0, 0, 255);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lighting.setLedBuffer(m_buffer);
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
