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
  public ArcadeDrive(Drivetrain drive, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drivetrain = drive;
    m_joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftRawInput = m_joystick.getRawAxis(Constants.Controller.JOYSTICK_1);
    double rightRawInput = m_joystick.getRawAxis(Constants.Controller.JOYSTICK_2);
    
    double leftFinalInput = 0;
    double rightFinalInput = 0;

    if (leftRawInput > Constants.Controller.DEAD_ZONE_SENSITIVITY && leftRawInput < 1) {
        if (leftRawInput < 1-Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
            leftFinalInput += Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
        }
        else{
            leftFinalInput = 1;
        }
    }
    if (leftRawInput < Constants.Controller.DEAD_ZONE_SENSITIVITY*-1 && leftRawInput > -1) {

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