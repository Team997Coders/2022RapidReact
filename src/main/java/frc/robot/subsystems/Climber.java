// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax m_motorCanSparkMax;

  public Climber() {
    m_motorCanSparkMax = new CANSparkMax(Constants.Ports.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
    m_motorCanSparkMax.restoreFactoryDefaults();
    m_motorCanSparkMax.setIdleMode(IdleMode.kBrake);
  }

  public void Move() {
    m_motorCanSparkMax.set(0.4);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
