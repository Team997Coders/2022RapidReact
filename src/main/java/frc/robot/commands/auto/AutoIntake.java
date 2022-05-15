// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
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

  /**
   * Runs the intake subsystem during auto.
   * Stops if the drawn current is too high (a ball has in fact been grabbed, or
   * something else is stuck).
   * 
   * @param intake   : A {@link SimpleIntake} subsystem.
   * @param pdp      : PDP to check for high current.
   * @param stall    : Current in amperes that triggers this to stop. Zero means
   *                 no threshold (not recommended).
   * @param timeMS   : Command will end after this many seconds, no matter if
   *                 was otherwise triggered or not. Set to 0 to never trigger
   *                 (not recommended).
   * @param inverted : Whether to run the motor backwards ("forwards" in hardware
   *                 cooresponds to a motion that will bring balls in).
   */
  public AutoIntake(Intake intake, PowerDistribution pdp, double stall, double timeS, boolean inverted) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_pdp = pdp;
    m_inverted = inverted;
    m_stall = stall;
    m_timeout = timeS;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_inverted)
      m_intake.setIntakeSpeed(-Constants.Intake.INTAKE_DEFAULT_SPEED);
    else
      m_intake.setIntakeSpeed(Constants.Intake.INTAKE_DEFAULT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // could stop if it draws too much current starting up
    return ((m_pdp.getCurrent(Constants.Ports.INTAKE_PDP) >= m_stall && m_stall != 0) ||
        ((Timer.getFPGATimestamp() - startTime) >= m_timeout && m_timeout != 0));
  }
}
