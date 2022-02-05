// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class JoystickClimber extends CommandBase {
  /** Creates a new JoystickClimber. */
  private Joystick js2;
  private Climber climber;
  public JoystickClimber(Climber climb) {
    addRequirements(climb);
    climber = climb;
    // Use addRequirements() here to declare subsystem dependencies. 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    js2 = new Joystick(Constants.Controller.CONTROLLER_1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climberMove(js2.getRawAxis(Constants.Controller.JOYSTICK_1));
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
