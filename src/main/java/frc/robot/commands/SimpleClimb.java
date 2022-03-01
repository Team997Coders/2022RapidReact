// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SimpleClimb extends CommandBase {
  /** Creates a new SimpleClimb. */
  private Climber m_climber;
  private Joystick m_joystick;
  private double movement;

  public SimpleClimb(Climber climb, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    m_climber = climb;
    m_joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    movement = -m_joystick.getRawAxis(Constants.Controller.TRIGGER_CLIMB_DN) + m_joystick.getRawAxis(Constants.Controller.TRIGGER_CLIMB_UP);
    if (Math.abs(movement) < 0.1) {movement = 0;}
    m_climber.climberMove(movement);
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
