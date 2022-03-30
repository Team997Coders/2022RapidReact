// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TimedDrive extends CommandBase {
  /** Creates a new TimedDrive. */
  private Drivetrain drive;
  private double duration;
  private double left;
  private double right;
  private double startTime;

  public TimedDrive(Drivetrain m_drive, double m_duration, double m_left, double m_right) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    drive = m_drive;
    duration = m_duration*1000;
    left = m_left;
    right = m_right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.basicMove(right, left);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()-startTime >= duration);
  }
}
