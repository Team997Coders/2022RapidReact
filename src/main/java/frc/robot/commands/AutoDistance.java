// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoDistance extends CommandBase {
  /** Creates a new TimedAutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private Constraints m_constraints;
  private double m_distance;
  private double measurement;
  private double timeout;
  private double startTime;
  public AutoDistance(Drivetrain drive, double distance, double timeSecs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    timeout = timeSecs*1000;
    m_constraints = new Constraints(Constants.MovementConstants.AUTO_DISTANCE_MAX_V, 
      Constants.MovementConstants.AUTO_DISTANCE_MAX_A);
    m_controller = new ProfiledPIDController(Constants.MovementConstants.AUTO_DISTANCE_KP, 
      Constants.MovementConstants.AUTO_DISTANCE_KI, 
      Constants.MovementConstants.AUTO_DISTANCE_KD, m_constraints);
    m_distance = distance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_controller.setGoal(m_distance);
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = (Drivetrain.frontRight.getSelectedSensorPosition()*Constants.DRIVE_IN_PER_COUNT
      + Drivetrain.frontLeft.getSelectedSensorPosition()*Constants.DRIVE_IN_PER_COUNT)/2;
    m_drive.tankDriveMove(m_controller.calculate(measurement), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (measurement-m_distance <= Constants.MovementConstants.AUTO_ROTATE_TOL*m_distance && 
    // measurement-m_distance >= -Constants.MovementConstants.AUTO_ROTATE_TOL*m_distance);
    return (m_controller.atGoal() || System.currentTimeMillis()>=(startTime+timeout));
  }
}
