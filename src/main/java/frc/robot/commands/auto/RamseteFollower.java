// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RamseteFollower extends CommandBase {
  /** Creates a new RamseteFollower. */
  private RamseteController ramseteController;
  private ProfiledPIDController rightVelocityController;
  private ProfiledPIDController leftVelocityController;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private Trajectory m_trajectory;

  private Drivetrain m_drive;

  private double startTime;
  private double m_timeoutS;

  private double m_homeXPos;
  private double m_homeYPos;
  private Rotation2d m_homeRotation;

  /**
   * A command containing a controller for a differential drive to navigate a
   * trajectory.
   * 
   * @param trajectory : The trajectory to follow.
   * @param drive      : The {@link Drivetrain} subsystem to use.
   * @param xPosition  : Starting X (field-relative) position of the robot when
   *                   this command runs.
   * @param yPosition  : Starting Y (field-relative) position of the robot when
   *                   this command runs.
   * @param rotation   : Starting rotation (field-relative) of the robot when this
   *                   command runs. Rotation2d is in radians, measured
   *                   counterclockwise from the positive X-axis.
   * @param timeoutS   Command will end after this many seconds, no matter if it
   *                   reached its goal or not. Set to 0 to never timeout (not
   *                   recommended).
   */
  public RamseteFollower(Trajectory trajectory, Drivetrain drive, double xPosition, double yPosition,
      Rotation2d rotation, double timeoutS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_trajectory = trajectory;
    m_drive = drive;

    m_homeXPos = xPosition;
    m_homeYPos = yPosition;
    m_homeRotation = rotation;

    ramseteController = new RamseteController(Constants.Drive.RAMSETE_B, Constants.Drive.RAMSETE_ZETA);

    rightVelocityController = new ProfiledPIDController(
        Constants.Drive.WHEEL_SPEEDS_KP, Constants.Drive.WHEEL_SPEEDS_KI,
        Constants.Drive.WHEEL_SPEEDS_KD,
        new Constraints(Constants.Drive.WHEELS_SPEEDS_MAX_A, Constants.Drive.WHEEL_SPEEDS_MAX_J));
    leftVelocityController = new ProfiledPIDController(
        Constants.Drive.WHEEL_SPEEDS_KP, Constants.Drive.WHEEL_SPEEDS_KI,
        Constants.Drive.WHEEL_SPEEDS_KD,
        new Constraints(Constants.Drive.WHEELS_SPEEDS_MAX_A, Constants.Drive.WHEEL_SPEEDS_MAX_J));

    kinematics = new DifferentialDriveKinematics(Constants.Drive.DRIVE_TRACK_WIDTH_METERS);

    addRequirements(drive);

    m_drive.getField().getObject("Trajectory").setTrajectory(m_trajectory);

    m_timeoutS = timeoutS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_drive.resetPose(m_homeXPos, m_homeYPos, m_homeRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp() - startTime;

    SmartDashboard.putNumber("time from trajectory start", time);

    wheelSpeeds = kinematics.toWheelSpeeds(ramseteController.calculate(
        m_drive.getPose(),
        m_trajectory.sample(time)));

    rightVelocityController.setGoal(
        wheelSpeeds.rightMetersPerSecond *
            Constants.Drive.DRIVE_ENCODER_CPR / Constants.Drive.DRIVE_GEARBOX_RATIO);
    leftVelocityController.setGoal(
        wheelSpeeds.leftMetersPerSecond *
            Constants.Drive.DRIVE_ENCODER_CPR / Constants.Drive.DRIVE_GEARBOX_RATIO);

    double rightControlEffort = rightVelocityController.calculate(
        m_drive.getRightSensorVelocity() * Constants.Drive.DRIVE_METERS_PER_COUNT);
    double leftControlEffort = leftVelocityController.calculate(
        -m_drive.getLeftSensorPosition() * Constants.Drive.DRIVE_METERS_PER_COUNT);

    m_drive.basicMove(rightControlEffort, leftControlEffort);

    SmartDashboard.putNumber("right control effort", rightControlEffort);
    SmartDashboard.putNumber("left control effort", leftControlEffort);
    SmartDashboard.putNumber("right commanded v", wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("left commanded v", wheelSpeeds.leftMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ramseteController.atReference()
        || (Timer.getFPGATimestamp() - startTime > m_timeoutS && m_timeoutS != 0));
  }
}
