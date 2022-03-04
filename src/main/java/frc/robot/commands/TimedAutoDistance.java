// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TimedAutoDistance extends CommandBase {
  /** Creates a new TimedAutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private Constraints m_constraints;
  private double length;
  private double startTime;
  private double m_distance;
  private double measurement;
  public TimedAutoDistance(Drivetrain drive, double duration, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_constraints = new Constraints(Constants.MovementConstants.AUTO_DISTANCE_MAX_V, 
      Constants.MovementConstants.AUTO_DISTANCE_MAX_A);
    m_controller = new ProfiledPIDController(Constants.MovementConstants.AUTO_DISTANCE_KP, 
      Constants.MovementConstants.AUTO_DISTANCE_KI, 
      Constants.MovementConstants.AUTO_DISTANCE_KD, m_constraints);
    length = duration*1000;
    startTime = System.currentTimeMillis();
    m_distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setGoal(m_distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = (m_drive.frontRight.getSelectedSensorPosition()*Constants.DRIVE_IN_PER_COUNT
      + m_drive.frontLeft.getSelectedSensorPosition()*Constants.DRIVE_IN_PER_COUNT)/2;
    m_drive.tankDriveMove(m_controller.calculate(measurement), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() >= startTime+length);
  }
}
