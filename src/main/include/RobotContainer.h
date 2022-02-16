// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include "subsystems/Drivetrain.h"
#include "commands/ArcadeDrive.h"
#include <frc2/command/Command.h>
#include "CustomAction.h"
#include "actions/ArcadeDriveAction.h"
#include "actions/ClimberMove.h"

class RobotContainer {
 public:
  RobotContainer();
  ~RobotContainer();
  frc2::Command* GetDefaultDriveCommand();
  CustomAction* GetDefaultDriveAction();

  private:
    frc::Joystick* m_joystick;
    Drivetrain* m_drivetrain;
    ArcadeDrive* m_defaultDriveCommand;
    ArcadeDriveAction* m_defaultDriveAction;
    ClimberMove* m_defaultClimerAction;
};
