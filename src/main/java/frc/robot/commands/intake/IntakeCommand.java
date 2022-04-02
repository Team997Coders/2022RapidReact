// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private Intake m_intake;
  private double m_intakeSpeed;
  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intakeSpeed = Constants.Intake.INTAKE_DEFAULT_SPEED;
  }
  public IntakeCommand(Intake intake, double intakeSpeed) {
    addRequirements(intake);
    m_intakeSpeed = intakeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeSpeed(m_intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
