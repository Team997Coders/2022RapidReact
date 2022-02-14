// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoRotate extends CommandBase {
  private Drivetrain drive;
  private double kP = 0.01; // temporary: replace with calls to constants after tuning done
  private double kI = 0;
  private double kD = 0;

  private double PIDOutput;
  private double rotTarget;
  private double rotDistance;
  private double startRot;
  /** Creates a new AutoDistance. */
  public AutoRotate(Drivetrain m_drivetrain, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drive = m_drivetrain;
    rotTarget = rotation;
  }

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.MovementConstants.DrivetrainConstants.DRIVE_ROT_CONSTRAINT_V,Constants.MovementConstants.DrivetrainConstants.DRIVE_ROT_CONSTRAINT_ACCEL);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(kP,kI,kD,m_constraints);
  
  public void reDisplayDriveRotPidGains () {
    SmartDashboard.putNumber("drive rot kP", kP);
    SmartDashboard.putNumber("drive rot kI", kI);
    SmartDashboard.putNumber("drive rot kD", kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber("drive rot kP", 0);
    kI = SmartDashboard.getNumber("drive rot kI", 0);
    kD = SmartDashboard.getNumber("drive rot kD", 0);

    m_controller.setPID(kP,kI,kD);
    m_controller.setGoal(rotTarget);

    startRot = drive.gyro.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotDistance = drive.gyro.getYaw()+startRot;

    PIDOutput = m_controller.calculate(rotDistance);
    drive.tankDriveMove(PIDOutput,0);

    SmartDashboard.putNumber("drive rot startRot", startRot);
    SmartDashboard.putNumber("drive rot error", m_controller.getSetpoint().position-rotDistance);
    SmartDashboard.putNumber("drive rot target", rotTarget);
    SmartDashboard.putNumber("drive rot pidOutput", PIDOutput);
    SmartDashboard.putNumber("drive rot setPoint", m_controller.getSetpoint().position);
    SmartDashboard.putNumber("drive rot setPoint v", m_controller.getSetpoint().velocity);
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
