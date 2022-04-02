// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class SimpleIntake extends CommandBase {
  /** Creates a new SimpleIntake. */
  Supplier<Boolean> m_forward, m_backward;
  Intake m_intake;

  public SimpleIntake(Intake intake, Supplier<Boolean> forward, Supplier<Boolean> backward) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_forward = forward;
    m_backward = backward;
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_backward.get()) {
      m_intake.setIntakeSpeed(-Constants.Intake.INTAKE_DEFAULT_SPEED);
    } else if (m_forward.get()) {
      m_intake.setIntakeSpeed(Constants.Intake.INTAKE_DEFAULT_SPEED);
    } else {
      m_intake.setIntakeSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
