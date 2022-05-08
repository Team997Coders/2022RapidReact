// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
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
  private double lastLeft, lastRight;
  private double currentLeft, currentRight;
  private Supplier<Double> m_xInput, m_zInput;
  private Supplier<Boolean> m_turbo;

  private Field2d m_field;
  private DifferentialDriveOdometry m_odometry;

  private double m_turnMod;
  private double m_driveMod;

  public ArcadeDrive(Drivetrain drive, Supplier<Double> xInput, Supplier<Double> zInput, Supplier<Boolean> turboButton, boolean demoMode) {
    // Use addRequirements() here to declare subsystem dependencies.

    if (demoMode) {
      m_turnMod = Constants.Drive.TURN_MODIFIER_DEMO;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_DEMO;
    } else {
      m_turnMod = Constants.Drive.TURN_MODIFIER_FULL;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_FULL;
    }

    addRequirements(drive);
    m_drivetrain = drive;
    m_turbo = turboButton;
    m_xInput = xInput;
    m_zInput = zInput;

    m_field = new Field2d();
    m_odometry = new DifferentialDriveOdometry(
        new Rotation2d(Math.toRadians(0)), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastRight = 0;
    lastLeft = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_odometry.update(new Rotation2d(Math.toRadians(m_drivetrain.getGyroAngle())), 
        m_drivetrain.getLeftSensorPosition()*Constants.Drive.DRIVE_IN_PER_COUNT/Constants.INCHES_PER_METER,
        m_drivetrain.getRightSensorPosition()*Constants.Drive.DRIVE_IN_PER_COUNT/Constants.INCHES_PER_METER);
    
    if (!m_turbo.get()) {
      currentLeft = (m_xInput.get() * m_driveMod) + (m_zInput.get() * m_turnMod);
      currentRight = (m_xInput.get() * m_driveMod) - (m_zInput.get() * m_turnMod);
    }

    if (Math.abs(currentLeft) <= Constants.Controller.DEAD_ZONE_SENSITIVITY) { lastLeft = 0; } 
    else { lastLeft = MathUtil.clamp(currentLeft, lastLeft - Constants.Drive.INPUT_SMOOTH_SLOPE, lastLeft + Constants.Drive.INPUT_SMOOTH_SLOPE); }
    if (Math.abs(currentRight) <= Constants.Controller.DEAD_ZONE_SENSITIVITY) { lastRight = 0; } 
    else { lastRight = MathUtil.clamp(currentRight, lastRight - Constants.Drive.INPUT_SMOOTH_SLOPE, lastRight + Constants.Drive.INPUT_SMOOTH_SLOPE); }
    
    m_drivetrain.basicMove(lastRight, lastLeft);
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