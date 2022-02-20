// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include "subsystems/Drivetrain.h"
#include "commands/ArcadeDrive.h"
#include <frc2/command/Command.h>

class RobotContainer {
 public:
  RobotContainer();
  ~RobotContainer();
  frc2::Command* GetDefaultDriveCommand();

  private:
    frc::Joystick* m_joystick;
    frc2::JoystickButton* m_turboButton;
    Drivetrain* m_drivetrain;
    ArcadeDrive* m_defaultDriveCommand;
};