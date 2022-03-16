// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SimpleClimb extends CommandBase {
  /** Creates a new SimpleClimb. */
  private Climber m_climber;
  private Supplier<Double> m_up, m_down;

  public SimpleClimb(Climber climb, Supplier<Double> up, Supplier<Double> down) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    m_climber = climb;
    m_up = up;
    m_down = down;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.climberMove(m_up.get() - m_down.get());
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
