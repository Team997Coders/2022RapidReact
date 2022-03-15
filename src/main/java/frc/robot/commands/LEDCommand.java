// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;

public class LEDCommand extends CommandBase {
  /** Creates a new LEDCommand. */
  private LEDs m_leds;
  private char ledMode;
  public LEDCommand(LEDs led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
    m_leds = led;
  }

  public void setLEDModeShine() {
    ledMode = 'S';
  }
  public void setLEDModeOff() {
    ledMode = 'O';
  }
  public void setLEDModeBlink() {
    ledMode = 'B';
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leds.m_led.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i<Constants.LED_COUNT; i++) {
      if (ledMode == 'O') {
        m_leds.m_ledBuffer.setRGB(i, 0, 0, 0);
      }
      else if (ledMode == 'S') {
        if (RobotContainer.allianceColor == "Red") {
          m_leds.m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        else {
          m_leds.m_ledBuffer.setRGB(i, 0, 0, 255);
        }
      }
      else if (ledMode == 'B') {
        if ((System.currentTimeMillis()/1000)%2 == 0) {
          m_leds.m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        else if (RobotContainer.allianceColor == "Red") {
          m_leds.m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        else {
          m_leds.m_ledBuffer.setRGB(i, 0, 0, 255);
        }
      }
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
