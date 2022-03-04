// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class LeaveTarmacAuto extends SequentialCommandGroup{
  /** Creates a new LeaveTarmacAuto. */
  public LeaveTarmacAuto(Drivetrain drivetrain) {
	// Use addRequirements() here to declare subsystem dependencies.
	addRequirements(drivetrain);
	addCommands(
	  new TimedAutoRotate(drivetrain, 4, 180),
	  new TimedAutoDistance(drivetrain, 4, 90),
	  new TimedDrive(drivetrain, 7, 0, 0)
	);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
	return false;
  }
}
