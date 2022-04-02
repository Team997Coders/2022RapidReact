// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.Drivetrain;
public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drivetrain m_drivetrain;
  private double lastLeft, lastRight;
  private double currentLeft, currentRight;
  private Supplier<Double> m_xInput, m_zInput;
  private Supplier<Boolean> m_turbo;
  public ArcadeDrive(Drivetrain drive, Supplier<Double> xInput, Supplier<Double> zInput, Supplier<Boolean> turboButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drivetrain = drive;
    m_turbo = turboButton;
    m_xInput = xInput;
    m_zInput = zInput;
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
    
    if (!m_turbo.get()) {
      currentLeft = (MathUtil.applyDeadband(m_xInput.get() * Constants.Drive.DRIVE_MODIFIER, Constants.Controller.DEAD_BAND) 
        + MathUtil.applyDeadband(m_zInput.get() * Constants.Drive.TURN_MODIFIER, Constants.Controller.DEAD_BAND));
      
        currentRight = (MathUtil.applyDeadband(m_xInput.get() * Constants.Drive.DRIVE_MODIFIER, Constants.Controller.DEAD_BAND) 
        - MathUtil.applyDeadband(m_zInput.get() * Constants.Drive.TURN_MODIFIER, Constants.Controller.DEAD_BAND));
    }

    //if (Math.abs(currentLeft) <= Constants.Controller.DEAD_ZONE_SENSITIVITY) { lastLeft = 0; } 
    //else { lastLeft = MathUtil.clamp(currentLeft, lastLeft - Constants.Drive.INPUT_SMOOTH_SLOPE, lastLeft + Constants.Drive.INPUT_SMOOTH_SLOPE); }
    //if (Math.abs(currentRight) <= Constants.Controller.DEAD_ZONE_SENSITIVITY) { lastRight = 0; } 
    //else { lastRight = MathUtil.clamp(currentRight, lastRight - Constants.Drive.INPUT_SMOOTH_SLOPE, lastRight + Constants.Drive.INPUT_SMOOTH_SLOPE); }
    
    lastLeft = MathUtil.clamp(currentLeft, lastLeft - Constants.Drive.INPUT_SMOOTH_SLOPE, lastLeft + Constants.Drive.INPUT_SMOOTH_SLOPE);
    lastRight = MathUtil.clamp(currentRight, lastRight - Constants.Drive.INPUT_SMOOTH_SLOPE, lastRight + Constants.Drive.INPUT_SMOOTH_SLOPE);

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