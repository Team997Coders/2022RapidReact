// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain drivetrain;
  public static Joystick js1;
  public ArcadeDrive(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    drivetrain = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    js1 = new Joystick(Constants.Ports.JOYSTICK_1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.betterArcadeDrive(-1*Constants.MovementConstants.LINEAR_SPEED*js1.getRawAxis(Constants.Ports.JOYSTICK_1), Constants.MovementConstants.TURN_SPEED*js1.getRawAxis(Constants.Ports.JOYSTICK_2));
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