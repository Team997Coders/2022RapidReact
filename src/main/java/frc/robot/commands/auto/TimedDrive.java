// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TimedDrive extends CommandBase {
  /** Creates a new TimedDrive. */
  private Drivetrain m_drive;
  private double timeout;
  private double m_left;
  private double m_right;
  private double startTime;

  public TimedDrive(Drivetrain drive, double left, double right, double timeoutMS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    timeout = timeoutMS;
    m_left = left;
    m_right = right;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.basicMove(m_right, m_left);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime >= timeout);
  }
}
