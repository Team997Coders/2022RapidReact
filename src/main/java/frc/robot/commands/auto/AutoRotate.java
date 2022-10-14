// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoRotate extends CommandBase {
  /** Creates a new TimedAutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private double m_rotation;
  private double timeout;
  private double startTime;

  /**
   * Command used in auto for simple turns to angles.
   * 
   * @param drive     : {@link Drivetrain} subsystem to use.
   * @param rotation  : Rotation (in degrees, clockwise) to target.
   * @param timeoutMS : Command will end after this many seconds, no matter
   *                  if it reached its goal or not. Set to 0 to never timeout
   *                  (not recommended).
   */
  public AutoRotate(Drivetrain drive, double rotation, double timeoutS) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    timeout = timeoutS;
    m_controller = new ProfiledPIDController(Constants.Drive.AUTO_ROTATE_KP,
        Constants.Drive.AUTO_ROTATE_KI,
        Constants.Drive.AUTO_ROTATE_KD,
        new Constraints(Constants.Drive.AUTO_ROTATE_MAX_V,
            Constants.Drive.AUTO_ROTATE_MAX_A));
    m_rotation = rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset(m_drive.getGyroAngle());
    startTime = Timer.getFPGATimestamp();
    m_controller.setGoal(m_drive.getGyroAngle() + m_rotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.basicMove(
        m_controller.calculate(m_drive.getGyroAngle()),
        -m_controller.calculate(m_drive.getGyroAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_controller.atGoal() || Timer.getFPGATimestamp() - startTime >= timeout && timeout != 0);
  }
}
