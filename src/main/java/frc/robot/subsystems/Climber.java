// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax m_motorCanSparkMax;
  private RelativeEncoder m_encoder;
  
  public Climber() { //constructer
    m_motorCanSparkMax = new CANSparkMax(Constants.Ports.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
    m_motorCanSparkMax.restoreFactoryDefaults();
    m_motorCanSparkMax.setIdleMode(IdleMode.kBrake);
    m_encoder = m_motorCanSparkMax.getEncoder();
    m_encoder.setPosition(0);

  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }

  public void Move(double speed) {
    m_motorCanSparkMax.set(speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", getEncoder());
  }
}
