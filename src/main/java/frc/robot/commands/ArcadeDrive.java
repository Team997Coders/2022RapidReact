// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain drivetrain;

  public ArcadeDrive(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    drivetrain = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public String smoothingChooser() {
    String deadManString = "Dead Band"; //       There was some confusion over whether this
    if (Constants.Other.deadManMode == true) {// was called "Dead Man" or "Dead Band".
      deadManString = "Dead Man";//              Why not both? Change in constants.
    }
    SendableChooser<String> smoothingChooser = new SendableChooser<>();

    smoothingChooser.setDefaultOption("Linear, "+deadManString, "LDB");
    smoothingChooser.addOption("Linear, No "+deadManString, "LNDB");
    smoothingChooser.addOption("Special, "+deadManString, "SDB");
    
    return smoothingChooser.getSelected();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    String selectedSmoothingOption = smoothingChooser();

    double leftRawInput = RobotContainer.js1.getRawAxis(Constants.Ports.JOYSTICK_1);
    double rightRawInput = RobotContainer.js1.getRawAxis(Constants.Ports.JOYSTICK_2);

    double leftSmoothedInput = leftRawInput;
    double rightSmoothedInput = rightRawInput;
    
    if (Math.abs(leftSmoothedInput)>1) {
      if (leftSmoothedInput<0) {
        leftSmoothedInput = -1;
      }
      else if (leftSmoothedInput!=0) {
        leftSmoothedInput = 1;
      }
    }
    if (Math.abs(rightSmoothedInput)>1) {
      if (rightSmoothedInput<0) {
        rightSmoothedInput = -1;
      }
      else if (rightSmoothedInput!=0) {
        rightSmoothedInput = 1;
      }
    }
    if (selectedSmoothingOption=="SDB"||selectedSmoothingOption=="LDB") {
      if (leftRawInput < Constants.MovementConstants.DEADBAND_TOLERANCE) {
        leftSmoothedInput = 0;
      }
      if (rightRawInput < Constants.MovementConstants.DEADBAND_TOLERANCE) {
        rightSmoothedInput = 0;
      }
    }
    else if (selectedSmoothingOption=="SDB") {

    }
    drivetrain.newArcadeDrive(-1*Constants.MovementConstants.LINEAR_SPEED*leftSmoothedInput, Constants.MovementConstants.TURN_SPEED*rightSmoothedInput);
    SmartDashboard.putNumber("lJoystickSmoothed", leftSmoothedInput);
    SmartDashboard.putNumber("rJoystickSmoothed", rightSmoothedInput);
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
