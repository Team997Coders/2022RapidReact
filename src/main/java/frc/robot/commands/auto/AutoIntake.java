// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {
  /** Creates a new AutoIntake. */
  private Intake m_intake;
  private PowerDistribution m_pdp;
  private double m_stall;
  private double m_timeout;
  private boolean m_inverted;
  
  private double startTime;

  public AutoIntake(Intake intake, PowerDistribution pdp, double stall, double timeMS, boolean inverted) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_pdp = pdp;
    m_inverted = inverted;
    m_stall = stall;
    m_timeout = timeMS;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_inverted) m_intake.setIntakeSpeed(-Constants.Intake.INTAKE_DEFAULT_SPEED);
    else m_intake.setIntakeSpeed(Constants.Intake.INTAKE_DEFAULT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // might automatically stop if it draws too much current starting up
    if (m_pdp.getCurrent(Constants.Ports.INTAKE_PDP) >= m_stall || System.currentTimeMillis() - startTime >= m_timeout) {
      return true;
    }
    return false;
  }
}
