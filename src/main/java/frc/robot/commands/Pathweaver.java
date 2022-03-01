// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.MovementConstants.DrivetrainConstants.PathweaverConstants;

public class Pathweaver extends CommandBase {
  /** Creates a new Pathweaver. */
  public Drivetrain m_drive;
  private RamseteController ramCont;
  private DifferentialDriveOdometry odometry;
  private Pose2d pose;
  private Trajectory autoTrajectory;
  private String filename;
  private DifferentialDriveKinematics kinematics;
  private PIDController rightSidePid;
  private PIDController leftSidePid;

  public Pathweaver(Drivetrain drive, String filepath) {
    addRequirements(drive);
    m_drive = drive;
    filename = filepath;
    kinematics = new DifferentialDriveKinematics(Constants.MovementConstants.DrivetrainConstants.DRIVE_WIDTH_METERS);
    ramCont = new RamseteController(PathweaverConstants.RAMSETE_B, PathweaverConstants.RAMSETE_ZETA);
    rightSidePid = new PIDController(PathweaverConstants.PW_PID_V_KP, PathweaverConstants.PW_PID_V_KI, PathweaverConstants.PW_PID_V_KD);
    leftSidePid = new PIDController(PathweaverConstants.PW_PID_V_KP, PathweaverConstants.PW_PID_V_KI, PathweaverConstants.PW_PID_V_KD);
  }

  public void robotInit() {
    try {
      Path autoPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      autoTrajectory = TrajectoryUtil.fromPathweaverJson(autoPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometry = new DifferentialDriveOdometry(new Rotation2d(m_drive.getGyroAngle()), autoTrajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    odometry.update(new Rotation2d(m_drive.getGyroAngle()), m_drive.getLeftDeltaDistanceM(), m_drive.getRightDeltaDistanceM());
    pose = odometry.getPoseMeters();
    ChassisSpeeds adjustedSpeeds = ramCont.calculate(pose, autoTrajectory.sample(Robot.m_timer.get()));

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
    rightSidePid.setSetpoint(wheelSpeeds.rightMetersPerSecond);
    leftSidePid.setSetpoint(wheelSpeeds.leftMetersPerSecond);

    m_drive.basicMove(
      rightSidePid.calculate(Drivetrain.frontRight.getSelectedSensorVelocity()),
      leftSidePid.calculate(Drivetrain.frontLeft.getSelectedSensorVelocity())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
