// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private VictorSPX intakeMotor;

  /**
   * Creates an intake subsystem with a single VictorSPX BAG motor.
   */
  public Intake() {
    intakeMotor = new VictorSPX(Constants.Ports.INTAKE_PORT);
    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE_DRAW_VOLTS);
    intakeMotor.enableVoltageCompensation(true);
    intakeMotor.setInverted(true);
  }

  /**
   * Sets the speed of the intake motor.
   * 
   * @param speed : value between [-1,1] (inclusive) to set the intake motor to. A
   *              positive value cooresponds to a motion that brings a ball in.
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
