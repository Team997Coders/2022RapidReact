// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BallDumpAuto extends SequentialCommandGroup {
  /** Creates a new BallDumpAuto. */
  public BallDumpAuto(Drivetrain drive, Intake intake, int leaveOrNo, int timeout) { // 1: move out after dump 0: stay after dump
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addCommands(
      new IntakeCommand(intake),
      new AutoDistance(drive, 36, 5),
      new TimedDrive(drive, 3, 0, 0),
      new AutoDistance(drive, -(96*leaveOrNo), timeout)
    );
  }
}
