// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoDistance extends CommandBase {
  /** Creates a new AutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private double m_distance;
  private double measurement;
  private double timeout;
  private double startTime;

  /**
   * Simple autonomous command for driving in straight lines.
   * 
   * @param drive          : {@link Drivetrain} subsystem to use.
   * @param distanceMeters : Distance to target.
   * @param timeoutMS      : Command will end after this many seconds, no
   *                       matter if it reached its goal or not. Set to 0 to never
   *                       timeout (not recommended).
   */
  public AutoDistance(Drivetrain drive, double distanceMeters, double timeoutS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    timeout = timeoutS;
    m_distance = distanceMeters;

    m_controller = new ProfiledPIDController(Constants.Drive.AUTO_DISTANCE_KP,
        Constants.Drive.AUTO_DISTANCE_KI,
        Constants.Drive.AUTO_DISTANCE_KD,
        new Constraints(Constants.Drive.AUTO_DISTANCE_MAX_V, Constants.Drive.AUTO_DISTANCE_MAX_A));

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset(0);
    m_drive.resetEncoders();
    m_controller.setGoal(m_distance);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = (m_drive.getRightSensorPosition() * Constants.Drive.DRIVE_METERS_PER_COUNT
        + m_drive.getLeftSensorPosition() * Constants.Drive.DRIVE_METERS_PER_COUNT) / 2;

    double controlEffort = m_controller.calculate(measurement);

    m_drive.basicMove(controlEffort, controlEffort);

    SmartDashboard.putNumber("control effort", controlEffort);
    SmartDashboard.putNumber("measurement", measurement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_controller.atGoal() || (Timer.getFPGATimestamp() - startTime >= timeout && timeout != 0));
  }
}
