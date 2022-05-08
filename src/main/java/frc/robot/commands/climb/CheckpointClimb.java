// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class CheckpointClimb extends CommandBase {
  /** Creates a new CheckpointClimb. */
  private Climber m_climber;
  private ProfiledPIDController controller;
  private int[] m_checkpoints;
  private int currentCheckpoint = 0;
  private Supplier<Boolean> m_upSupplier;
  private Supplier<Boolean> m_downSupplier;
  private LinearFilter voltageFilter;
  public CheckpointClimb(Climber climber, Supplier<Boolean> upButtonSupplier, Supplier<Boolean> downButtonSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    controller = new ProfiledPIDController(Constants.Climber.CLIMB_LOW_PID_KP, 
      Constants.Climber.CLIMB_LOW_PID_KI, Constants.Climber.CLIMB_LOW_PID_KD, 
        new Constraints(Constants.Climber.CLIMB_LOW_V_LIM, Constants.Climber.CLIMB_LOW_A_LIM));
    controller.setTolerance(Constants.Climber.CLIMB_SETPOINT_TOLERANCE);
    m_checkpoints = Constants.Climber.CLIMB_CHECKPOINTS;
    m_upSupplier = upButtonSupplier;
    m_downSupplier = downButtonSupplier;
    voltageFilter = LinearFilter.movingAverage(Constants.Climber.CLIMB_FILTER_TAPS);
    addRequirements(climber);
  }

  public void nextCheckpoint(boolean overrideSequence) {
    if (overrideSequence || controller.atSetpoint()) {
      if (currentCheckpoint < m_checkpoints.length-1) {
        currentCheckpoint += 1;
        controller.setGoal(m_checkpoints[currentCheckpoint]);
      }
    }
  }

  public void previousCheckpoint(boolean overrideSequence) {
    if (overrideSequence || controller.atSetpoint()) {
      if (currentCheckpoint > 0) {
        currentCheckpoint -= 1;
        controller.setGoal(m_checkpoints[currentCheckpoint]);
      }
    }
  }

  public boolean getHighGains() {
    return (voltageFilter.calculate(m_climber.getMotorCurrent()) > 30);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_upSupplier.get() && !m_downSupplier.get()) {
      nextCheckpoint(false);
    } else if (m_downSupplier.get() && !m_upSupplier.get()) {
      previousCheckpoint(true);
      previousCheckpoint(false);
    }
    
    double PIDCalculatedValue;
    if (getHighGains()) {
      controller.setPID(Constants.Climber.CLIMB_HIGH_PID_KP, Constants.Climber.CLIMB_HIGH_PID_KI, 
        Constants.Climber.CLIMB_HIGH_PID_KD);
      controller.setConstraints(new Constraints(Constants.Climber.CLIMB_HIGH_V_LIM, 
        Constants.Climber.CLIMB_HIGH_A_LIM));
    } else {
      controller.setPID(Constants.Climber.CLIMB_LOW_PID_KP, Constants.Climber.CLIMB_LOW_PID_KI, 
        Constants.Climber.CLIMB_LOW_PID_KD);
      controller.setConstraints(new Constraints(Constants.Climber.CLIMB_LOW_V_LIM,
        Constants.Climber.CLIMB_LOW_A_LIM));
    }

    PIDCalculatedValue = controller.calculate(m_climber.getEncoderPosition());

    m_climber.climberMove(-PIDCalculatedValue, false);
    
    SmartDashboard.putNumber("encoder", m_climber.getEncoderPosition());
    SmartDashboard.putNumber("error", controller.getSetpoint().position - m_climber.getEncoderPosition());
    SmartDashboard.putNumber("climb phase", currentCheckpoint);
    SmartDashboard.putNumber("output", PIDCalculatedValue);
    SmartDashboard.putBoolean("voltage high", getHighGains());
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
