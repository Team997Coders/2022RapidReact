// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Climber.h"
#include "commands/ArcadeDrive.h"
#include "commands/ClimberMove.h"
#include <frc2/command/Command.h>

class RobotContainer {
 public:
  RobotContainer();
  ~RobotContainer();
  frc2::Command* GetDefaultDriveCommand();
  frc2::Command* GetDefaultClimberCommand();
  Drivetrain* GetDrivetrain();
  Climber* GetClimber();
  private:
    frc::Joystick* m_joystick1;
    frc2::JoystickButton* m_turboButton;
    Drivetrain* m_drivetrain;
    Climber* m_climber;
    ArcadeDrive* m_defaultDriveCommand;
    ClimberMove* m_defaultClimberCommand;
};
