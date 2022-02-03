// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private Joystick js1;
  private static final int deviceID = 1;
  private CANSparkMax m_motorCanSparkMax;

  public void teleopPeriodic(){
    m_motorCanSparkMax.set(js1.getY());
  }
  public Climber() {
    m_motorCanSparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motorCanSparkMax.restoreFactoryDefaults();
    js1 = new Joystick(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
