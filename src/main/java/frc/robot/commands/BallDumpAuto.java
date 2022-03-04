// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BallDumpAuto extends SequentialCommandGroup {
  /** Creates a new BallDumpAuto. */
  public BallDumpAuto(Drivetrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    addCommands(
      new TimedDrive(m_drive, 0.5, -0.2, -0.2),
      new TimedDrive(m_drive, 0.25, 0, 0),
      new TimedDrive(m_drive, 0.25, 0.8, 0.8),
      // new TimeLimitedAutoDistance(m_drive, 3, 2) // meters, seconds
      new TimedDrive(m_drive, 1, -0.4, -0.4),
      // new TimeLimitedAutoRotate(m_drive, 180, 2) // degrees, seconds
      new TimedDrive(m_drive, 1, 0.2, -0.2)
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
