// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BallDumpAuto extends SequentialCommandGroup {
  /** Creates a new BallDumpAuto. */
  public BallDumpAuto(Drivetrain m_drive, int mode) { // 1: move out after dump 0: stay after dump
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    addCommands(
      new AutoDistance(m_drive, 36, 5),
      new TimedDrive(m_drive, 3, 0, 0),
      new AutoDistance(m_drive, (-96*mode), 5)
    );
  }
}
