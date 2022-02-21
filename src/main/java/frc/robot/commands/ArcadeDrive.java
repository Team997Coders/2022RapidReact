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
  private Drivetrain m_drivetrain;
  private Joystick m_joystick;
  private double lastLeft;
  private double lastRight;
  public ArcadeDrive(Drivetrain drive, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drivetrain = drive;
    m_joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastRight = 0;
    lastLeft = 0;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double left = m_joystick.getRawAxis(Constants.Controller.JOYSTICK_1)*Constants.MovementConstants.DRIVE_MODIFIER + m_joystick.getRawAxis(Constants.Controller.JOYSTICK_2)*Constants.MovementConstants.TURN_MODIFIER;
    double right = m_joystick.getRawAxis(Constants.Controller.JOYSTICK_1)*Constants.MovementConstants.DRIVE_MODIFIER - m_joystick.getRawAxis(Constants.Controller.JOYSTICK_2)*Constants.MovementConstants.TURN_MODIFIER;

    if (left-lastLeft >= Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
      left = lastLeft + Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
    }
    else if (left-lastLeft<=-Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
      left = lastLeft - Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
    }
    
    if (right-lastRight >=Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
      right = lastRight + Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
    }
    else if (right-lastRight<=-Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
      right = lastRight - Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
    }
    m_drivetrain.basicMove(right, left);
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