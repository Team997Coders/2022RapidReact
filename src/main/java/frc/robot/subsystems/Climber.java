// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
  private Climber climber;
  private DigitalInput m_zero;

  public Climber() { //constructer
    m_motorCanSparkMax = new CANSparkMax(Constants.Ports.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
    m_motorCanSparkMax.restoreFactoryDefaults();
    m_motorCanSparkMax.setIdleMode(IdleMode.kBrake);
    m_encoder = m_motorCanSparkMax.getEncoder();
    m_encoder.setPosition(0);
    m_zero = new DigitalInput(0);

  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }
  
  public boolean ZeroSwitch() {
    return !m_zero.get();
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }

  public void Move(double speed) {
    double EncoderPosition = getEncoder();
    if (EncoderPosition > Constants.Ports.MAXIMUM_EXTENSION && speed > 0) {
      m_motorCanSparkMax.set(0);
    }
    else if (EncoderPosition < Constants.Ports.MINIMUM_EXTENSION && speed < 0) {
      m_motorCanSparkMax.set(0);
    }
    else {
      m_motorCanSparkMax.set(speed);
    }
    if (ZeroSwitch() == true) {
      m_motorCanSparkMax.set(0);
      m_encoder.setPosition(0);
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", getEncoder());
  }
}
