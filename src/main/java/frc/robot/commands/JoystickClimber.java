// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class JoystickClimber extends CommandBase {
  /** Creates a new JoystickClimber. */
  private Climber climber;
  public JoystickClimber(Climber climb) {
    addRequirements(climb);
    climber = climb;
    // Use addRequirements() here to declare subsystem dependencies. 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double joystickPosition = -1*RobotContainer.js1.getRawAxis(Constants.Controller.JOYSTICK_1);
    SmartDashboard.putNumber("joystickPosition", joystickPosition);
    if (joystickPosition < 0) {
      if (Climber.climberEncoder.getPosition() >= Constants.MovementConstants.CLIMBER_MIN_HEIGHT) {
        climber.climberMove(Constants.MovementConstants.CLIMBER_MOTOR_SPEED*joystickPosition);
      }
    }
    else if (joystickPosition > 0) {
      if (Climber.climberEncoder.getPosition() < Constants.MovementConstants.CLIMBER_MAX_HEIGHT) {
        climber.climberMove(Constants.MovementConstants.CLIMBER_MOTOR_SPEED*joystickPosition);
      }
    }
    else if (joystickPosition == 0) {
      climber.climberMove(0);
    }
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
