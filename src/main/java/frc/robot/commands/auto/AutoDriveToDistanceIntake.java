// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveToDistanceIntake extends ParallelCommandGroup {
  /** Creates a new DriveToDistanceIntake. */
  public AutoDriveToDistanceIntake(Drivetrain drive, Intake intake, PowerDistribution pdp, double distance, double timeMS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDistance(drive, distance, timeMS),
      new AutoIntake(intake, pdp, 52, timeMS, false) // 53 is stall current
    );
  }
}
