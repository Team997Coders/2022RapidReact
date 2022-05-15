// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import frc.robot.Constants.Drive.DriveCommandModes;
import frc.robot.Constants.Controller.JoystickMappingTypes;

public class TeleopDriveCommand extends CommandBase {
  private Drivetrain m_drivetrain;
  private Supplier<Double> m_leftStick, m_rightStick;

  private double m_turnMod;
  private double m_driveMod;

  private Supplier<Boolean> m_demoModeSupplier;
  private Boolean m_isReal;

  private Supplier<DriveCommandModes> m_controlModeSupplier;
  private Supplier<JoystickMappingTypes> m_joystickMappingSupplier;

  private SlewRateLimiter primaryTurnLimiter;
  private SlewRateLimiter primaryDriveLimiter;

  private SlewRateLimiter secondaryTurnLimiter;
  private SlewRateLimiter secondaryDriveLimiter;

  /**
   * This command contains modes for controlling the robot by Arcade, Curvature,
   * or a mixture of the
   * methods.
   * 
   * @param drive                   : A {@link Drivetrain} subsystem.
   * @param leftStick               : A supplier for real-time values of the left
   *                                joystick of a controller.
   * @param rightStick              : A supplier for real-time values of the right
   *                                joystick of a controller.
   * @param demoMode                : A supplier for whether to use reduced gains
   *                                (for
   *                                demonstrations).
   * @param isReal                  : True if this code is running on a RoboRio,
   *                                false
   *                                if running in simulation.
   * @param controlModeSupplier     : A supplier for the {@link DriveCommandModes}
   *                                to
   *                                use.
   * @param joystickMappingSupplier : A supplier for the
   *                                {@link JoystickMappingTypes} to use.
   */
  public TeleopDriveCommand(
      Drivetrain drive, Supplier<Double> leftStick, Supplier<Double> rightStick,
      Supplier<Boolean> demoMode, Boolean isReal,
      Supplier<DriveCommandModes> controlModeSupplier, Supplier<JoystickMappingTypes> joystickMappingSupplier) {

    primaryTurnLimiter = new SlewRateLimiter(Constants.Drive.ROT_SLEW_LIMIT);
    primaryDriveLimiter = new SlewRateLimiter(Constants.Drive.LIN_SLEW_LIMIT);
    secondaryTurnLimiter = new SlewRateLimiter(Constants.Drive.ROT_SLEW_LIMIT);
    secondaryDriveLimiter = new SlewRateLimiter(Constants.Drive.LIN_SLEW_LIMIT);

    m_demoModeSupplier = demoMode;
    m_isReal = isReal;
    m_controlModeSupplier = controlModeSupplier;
    m_joystickMappingSupplier = joystickMappingSupplier;

    addRequirements(drive);
    m_drivetrain = drive;
    m_leftStick = leftStick;
    m_rightStick = rightStick;
  }

  /**
   * Maps raw inputs to processed values, clamped between [-1,1] (inclusive).
   * <p>
   * {@link JoystickMappingTypes#LINEAR} simply returns the input.
   * <p>
   * {@link JoystickMappingTypes#QUADRATIC} squares the input, and then reapplies
   * the sign.
   * 
   * @param mappingType : The method by which to process.
   * @param input       : The input to process.
   * @return : The processed values.
   */
  private double mapRawInputs(JoystickMappingTypes mappingType, double input) {
    if (mappingType == JoystickMappingTypes.LINEAR) {
      return MathUtil.clamp(input, -1, 1);
    } else if (mappingType == JoystickMappingTypes.QUADRATIC) {
      return MathUtil.clamp((input * input) * Math.signum(input), -1, 1);
    } else {
      return 0;
    }
  }

  /**
   * Moves the robot in teleoperated mode using separate inputs for linear and
   * rotational movement.
   * 
   * @param linInput : value between [-1,1] (inclusive) cooresponding with linear
   *                 motion forwards
   * @param rotInput : value between [-1,1] (inclusive) cooresponding with
   *                 rotational motion clockwise
   */
  private void arcadeDriveExecute(double linInput, double rotInput) {
    if (!m_demoModeSupplier.get() || !m_isReal || DriverStation.isFMSAttached()) {
      m_turnMod = Constants.Drive.TURN_MODIFIER_FULL;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_FULL;
    } else {
      m_turnMod = Constants.Drive.TURN_MODIFIER_DEMO;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_DEMO;
    }

    // right = linear + rotational
    // left = linear - rotational
    m_drivetrain.basicMove(
        primaryDriveLimiter.calculate(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod -
            primaryTurnLimiter
                .calculate(MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND)) * m_turnMod,
        primaryDriveLimiter.calculate(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod +
            primaryTurnLimiter
                .calculate(MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND)) * m_turnMod);
  }

  /**
   * Moves the robot in teleoperated mode similarly to
   * {@link #arcadeDriveExecute}, but with turning speeds dependent on linear
   * speeds.
   * 
   * @param linInput : value between [-1,1] (inclusive) cooresponding with linear
   *                 motion forwards
   * @param rotInput : value between [-1,1] (inclusive) cooresponding with
   *                 rotational motion clockwise
   */
  private void curvatureDriveExecute(double linInput, double rotInput) {
    if (!m_demoModeSupplier.get() || !m_isReal || DriverStation.isFMSAttached()) {
      m_turnMod = Constants.Drive.TURN_MODIFIER_FULL;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_FULL;
    } else {
      m_turnMod = Constants.Drive.TURN_MODIFIER_DEMO;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_DEMO;
    }

    // right = linear + (|linear| * rotational)
    // left = linear - (|linear| * rotational)
    m_drivetrain.basicMove(
        primaryDriveLimiter.calculate(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod -
            m_turnMod
                * primaryTurnLimiter
                    .calculate((Math.abs(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) *
                        MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND))),
        primaryDriveLimiter.calculate(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod +
            m_turnMod
                * primaryTurnLimiter
                    .calculate((Math.abs(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) *
                        MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND))));
  }

  /**
   * Mixture of {@link #arcadeDriveExecute} and {@link #curvatureDriveExecute}.
   * Proportions between the two set in Constants.
   * 
   * @param linInput : value between [-1,1] (inclusive) cooresponding with linear
   *                 motion forwards
   * @param rotInput : value between [-1,1] (inclusive) cooresponding with
   *                 rotational motion clockwise
   */
  private void mixedArcadeCurvatureExecute(double linInput, double rotInput) {
    if (!m_demoModeSupplier.get() || !m_isReal || DriverStation.isFMSAttached()) {
      m_turnMod = Constants.Drive.TURN_MODIFIER_FULL;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_FULL;
    } else {
      m_turnMod = Constants.Drive.TURN_MODIFIER_DEMO;
      m_driveMod = Constants.Drive.DRIVE_MODIFIER_DEMO;
    }

    // curvature * (1-arcade proportion) + arcade * arcade proportion
    double curvatureRightResult = primaryDriveLimiter
        .calculate(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod -
        m_turnMod * primaryTurnLimiter
            .calculate((Math.abs(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) *
                MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND)));
    double arcadeRightResult = secondaryDriveLimiter.calculate(
        MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod -
        secondaryTurnLimiter
            .calculate(MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND)) * m_turnMod;
    double curvatureLeftResult = primaryDriveLimiter
        .calculate(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod +
        m_turnMod * primaryTurnLimiter
            .calculate((Math.abs(MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) *
                MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND)));
    double arcadeLeftResult = secondaryDriveLimiter.calculate(
        MathUtil.applyDeadband(linInput, Constants.Controller.DEAD_BAND)) * m_driveMod +
        secondaryTurnLimiter
            .calculate(MathUtil.applyDeadband(rotInput, Constants.Controller.DEAD_BAND)) * m_turnMod;

    m_drivetrain.basicMove(
        ((1 - Constants.Drive.MIXED_PROPORTION_ARCADE) * curvatureRightResult) +
            Constants.Drive.MIXED_PROPORTION_ARCADE * arcadeRightResult,
        ((1 - Constants.Drive.MIXED_PROPORTION_ARCADE) * curvatureLeftResult) +
            Constants.Drive.MIXED_PROPORTION_ARCADE * arcadeLeftResult);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controlModeSupplier.get() == DriveCommandModes.ARCADE_DRIVE) {
      arcadeDriveExecute(mapRawInputs(m_joystickMappingSupplier.get(), m_leftStick.get()),
          mapRawInputs(m_joystickMappingSupplier.get(), m_rightStick.get()));
    } else if (m_controlModeSupplier.get() == DriveCommandModes.CURVATURE_DRIVE) {
      curvatureDriveExecute(mapRawInputs(m_joystickMappingSupplier.get(), m_leftStick.get()),
          mapRawInputs(m_joystickMappingSupplier.get(), m_rightStick.get()));
    } else if (m_controlModeSupplier.get() == DriveCommandModes.MIXED_ARCADE_CURVATURE) {
      mixedArcadeCurvatureExecute(mapRawInputs(m_joystickMappingSupplier.get(), m_leftStick.get()),
          mapRawInputs(m_joystickMappingSupplier.get(), m_rightStick.get()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}