// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoRotate extends CommandBase {
  /** Creates a new TimedAutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private Constraints m_constraints;
  private double m_rotation;
  private double measurement;
  public AutoRotate(Drivetrain drive, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_constraints = new Constraints(Constants.MovementConstants.AUTO_ROTATE_MAX_V, 
      Constants.MovementConstants.AUTO_ROTATE_MAX_A);
    m_controller = new ProfiledPIDController(Constants.MovementConstants.AUTO_ROTATE_KP, 
      Constants.MovementConstants.AUTO_ROTATE_KI, 
      Constants.MovementConstants.AUTO_ROTATE_KD, m_constraints);
    m_rotation = rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setGoal(m_rotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = ((Drivetrain.gyro.getYaw())/2)%360;
    m_drive.tankDriveMove(0, m_controller.calculate(measurement));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (measurement-m_rotation <= Constants.MovementConstants.AUTO_ROTATE_TOL*m_rotation && 
      measurement-m_rotation >= -Constants.MovementConstants.AUTO_ROTATE_TOL*m_rotation);
  }
}
