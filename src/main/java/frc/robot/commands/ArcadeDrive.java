// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain m_drivetrain;
  private Joystick m_joystick;
  private double leftFinalInput = 0;
  private double rightFinalInput = 0;
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

    SmartDashboard.putNumber("leftRawInput", leftRawInput);
    SmartDashboard.putNumber("rightRawInput", rightRawInput);
    SmartDashboard.putNumber("leftFinalInput", leftFinalInput);
    SmartDashboard.putNumber("rightFinalInput", rightFinalInput);

    if (leftRawInput > Constants.Controller.DEAD_ZONE_SENSITIVITY && leftRawInput < 1) {
        if (leftRawInput < 1-Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
            leftFinalInput += Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
        }
        else {
            leftFinalInput = 1;
        }
    }
    if (rightRawInput < Constants.Controller.DEAD_ZONE_SENSITIVITY*-1 && rightRawInput > -1) {
        if (rightRawInput < 1-Constants.MovementConstants.INPUT_SMOOTH_SLOPE) {
            rightFinalInput += Constants.MovementConstants.INPUT_SMOOTH_SLOPE;
        }
        else {
            rightFinalInput = 1;
        }
    }
    m_drivetrain.tankDriveMove(leftFinalInput, rightFinalInput);
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