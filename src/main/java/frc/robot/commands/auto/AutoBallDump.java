// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoBallDump extends SequentialCommandGroup {
  /** Creates a new BallDumpAuto. */
  public AutoBallDump(Drivetrain drive, Intake intake, double endingDistance, int timeout) { // 1: move out after dump 0: stay after dump
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addCommands(
      new IntakeCommand(intake),
      new AutoDistance(drive, 36, 5000),
      new TimedDrive(drive, 0, 0, 3000),
      new AutoDistance(drive, endingDistance, timeout)
    );
  }
}
