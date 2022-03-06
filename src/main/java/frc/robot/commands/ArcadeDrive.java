// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain m_drivetrain;
  private double lastLeft;
  private double lastRight;
  private double slope;
  private double driveMod;
  private double turnMod;
  private boolean turbo;
  public ArcadeDrive(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drivetrain = drive;
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
    
    turbo = RobotContainer.turboModePressed();
    if (turbo) {
      slope = Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
      driveMod = Constants.MovementConstants.DRIVE_MODIFIER;
    } else {
      slope = Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
      driveMod = Constants.MovementConstants.DRIVE_MODIFIER;
    }
    turnMod = Constants.MovementConstants.TURN_MODIFIER;
    
    double left = RobotContainer.joystickLeftInput()*driveMod + RobotContainer.joystickRightInput()*turnMod;
    double right = RobotContainer.joystickLeftInput()*driveMod - RobotContainer.joystickRightInput()*turnMod;
    lastLeft = MathUtil.clamp(left, lastLeft-slope, lastLeft+slope);
    lastRight = MathUtil.clamp(right, lastRight-slope, lastRight+slope);
    m_drivetrain.basicMove(lastRight, lastLeft);
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