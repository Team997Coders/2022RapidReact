// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain m_drivetrain;
  private Supplier<Double> m_xInput, m_zInput;

  private Field2d m_field;
  private DifferentialDriveOdometry m_odometry;

  private double m_turnMod;
  private double m_driveMod;

  private SlewRateLimiter turnLimiter;
  private SlewRateLimiter driveLimiter;

  public ArcadeDrive(Drivetrain drive, Supplier<Double> xInput, Supplier<Double> zInput, boolean demoMode) {
    // Use addRequirements() here to declare subsystem dependencies.

    turnLimiter = new SlewRateLimiter(Constants.Drive.ROT_SLEW_LIMIT);
    driveLimiter = new SlewRateLimiter(Constants.Drive.LIN_SLEW_LIMIT);

    if (demoMode) {
      m_turnMod = Constants.Drive.TURN_MODIFIER_DEMO;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_DEMO;
    } else {
      m_turnMod = Constants.Drive.TURN_MODIFIER_FULL;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_FULL;
    }

    addRequirements(drive);
    m_drivetrain = drive;
    m_xInput = xInput;
    m_zInput = zInput;

    m_field = new Field2d();
    m_odometry = new DifferentialDriveOdometry(
        new Rotation2d(Math.toRadians(0)), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_odometry.update(new Rotation2d(Math.toRadians(m_drivetrain.getGyroAngle())), 
        m_drivetrain.getLeftSensorPosition()*Constants.Drive.DRIVE_IN_PER_COUNT/Constants.INCHES_PER_METER,
        m_drivetrain.getRightSensorPosition()*Constants.Drive.DRIVE_IN_PER_COUNT/Constants.INCHES_PER_METER);

    m_drivetrain.basicMove(
      driveLimiter.calculate(MathUtil.applyDeadband(m_xInput.get(), Constants.Controller.DEAD_BAND))*m_driveMod +
        turnLimiter.calculate(MathUtil.applyDeadband(m_zInput.get(), Constants.Controller.DEAD_BAND))*m_turnMod,
      driveLimiter.calculate(MathUtil.applyDeadband(m_xInput.get(), Constants.Controller.DEAD_BAND))*m_driveMod -
        turnLimiter.calculate(MathUtil.applyDeadband(m_zInput.get(), Constants.Controller.DEAD_BAND))*m_turnMod);

    SmartDashboard.putData("Field", m_field);
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