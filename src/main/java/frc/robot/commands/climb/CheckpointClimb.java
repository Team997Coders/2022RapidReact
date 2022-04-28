// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class CheckpointClimb extends CommandBase {
  /** Creates a new CheckpointClimb. */
  private Climber m_climber;
  private PIDController controller;
  private int[] m_checkpoints;
  private int currentCheckpoint = 0;
  private Supplier<Boolean> m_upSupplier;
  private Supplier<Boolean> m_downSupplier;
  public CheckpointClimb(Climber climber, Supplier<Boolean> upButtonSupplier, Supplier<Boolean> downButtonSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    controller = new PIDController(Constants.Climber.CLIMB_PID_KP, 
        Constants.Climber.CLIMB_PID_KI, Constants.Climber.CLIMB_PID_KD);
    m_checkpoints = Constants.Climber.CLIMB_CHECKPOINTS;
    m_upSupplier = upButtonSupplier;
    m_downSupplier = downButtonSupplier;
    addRequirements(climber);
  }

  public void nextCheckpoint(boolean overrideSequence) {
    if (overrideSequence || controller.atSetpoint()) {
      if (currentCheckpoint < m_checkpoints.length-2) {
        currentCheckpoint += 1;
        controller.setSetpoint(m_checkpoints[currentCheckpoint]);
      }
    }
  }

  public void previousCheckpoint(boolean overrideSequence) {
    if (overrideSequence || controller.atSetpoint()) {
      if (currentCheckpoint > 0) {
        currentCheckpoint -= 1;
        controller.setSetpoint(m_checkpoints[currentCheckpoint]);
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(m_climber.getEncoderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.getZeroSwitch()) {
      currentCheckpoint = 0;
    }

    if (m_upSupplier.get() && !m_downSupplier.get()) {
      nextCheckpoint(false);
    } else if (m_downSupplier.get() && !m_upSupplier.get()) {
      previousCheckpoint(false);
    }
    
    m_climber.climberMove(controller.calculate(m_climber.getEncoderPosition()), false);
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
