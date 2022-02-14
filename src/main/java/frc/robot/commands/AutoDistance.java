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

public class AutoDistance extends CommandBase {
  private Drivetrain drive;
  private double kP = 0.01; // temporary: replace with calls to constants after tuning done
  private double kI = 0;
  private double kD = 0;

  private double PIDOutput;
  private double targetDistance;
  private double linDistance;

  /** Creates a new AutoDistance. */
  public AutoDistance(Drivetrain m_drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drive = m_drivetrain;
    targetDistance = distance;
  }

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.MovementConstants.DrivetrainConstants.DRIVE_LIN_CONSTRAINT_V,Constants.MovementConstants.DrivetrainConstants.DRIVE_LIN_CONSTRAINT_ACCEL);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(kP,kI,kD,m_constraints);
  
  public void reDisplayDriveLinPidGains () {
    SmartDashboard.putNumber("drive lin kP", kP);
    SmartDashboard.putNumber("drive lin kI", kI);
    SmartDashboard.putNumber("drive lin kD", kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber("drive lin kP", 0);
    kI = SmartDashboard.getNumber("drive lin kI", 0);
    kD = SmartDashboard.getNumber("drive lin kD", 0);

    m_controller.setPID(kP,kI,kD);
    m_controller.setGoal(targetDistance);

    drive.resetEncoders();
    linDistance = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    linDistance = ((Drivetrain.frontLeft.getSelectedSensorPosition()+Drivetrain.frontLeft.getSelectedSensorPosition())/2)*Constants.MovementConstants.DrivetrainConstants.ENCODER_TO_DISTANCE_FACTOR;
    
    PIDOutput = m_controller.calculate(linDistance);
    drive.tankDriveMove(PIDOutput,0);

    SmartDashboard.putNumber("drive lin error", m_controller.getSetpoint().position-linDistance);
    SmartDashboard.putNumber("drive lin target", targetDistance);
    SmartDashboard.putNumber("drive lin pidOutput", PIDOutput);
    SmartDashboard.putNumber("drive lin setPoint", m_controller.getSetpoint().position);
    SmartDashboard.putNumber("drive lin setPoint v", m_controller.getSetpoint().velocity);
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
