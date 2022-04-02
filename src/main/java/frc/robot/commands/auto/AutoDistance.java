// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoDistance extends CommandBase {
  /** Creates a new TimedAutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private double m_distance;
  private double measurement;
  private double timeout;
  private double startTime;
  
  public AutoDistance(Drivetrain drive, double distance, double timeoutMS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    timeout = timeoutMS;
    m_distance = distance; // flip because drive is inverted

    m_controller = new ProfiledPIDController(Constants.Drive.AUTO_DISTANCE_KP, 
      Constants.Drive.AUTO_DISTANCE_KI, 
      Constants.Drive.AUTO_DISTANCE_KD,
      new Constraints(Constants.Drive.AUTO_DISTANCE_MAX_V, Constants.Drive.AUTO_DISTANCE_MAX_A));
    
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
    measurement = (m_drive.getRightSensorPosition() * Constants.Drive.DRIVE_IN_PER_COUNT 
      + m_drive.getLeftSensorPosition() * Constants.Drive.DRIVE_IN_PER_COUNT) / 2;
    
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
    return (m_controller.atGoal() || System.currentTimeMillis() - startTime >= timeout);
  }
}
