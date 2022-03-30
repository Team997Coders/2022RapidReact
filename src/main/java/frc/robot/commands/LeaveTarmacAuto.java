// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class LeaveTarmacAuto extends SequentialCommandGroup{
  /** Creates a new LeaveTarmacAuto. */
  public LeaveTarmacAuto(Drivetrain drivetrain, int mode) { // 0: positioned on side 1: positioned in center
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new AutoDistance(drivetrain, (-60+(mode*-30)), 15)
    );
  }
}
