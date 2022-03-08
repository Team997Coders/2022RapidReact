// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/Command.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Climber.h"
#include "commands/ArcadeDrive.h"
#include "commands/ClimberMove.h"
#include "commands/AutoTurnAngle.h"
#include "commands/AutoDriveForward.h"
#include "commands/DoNothing.h"


class RobotContainer {
 public:
  RobotContainer();
  ~RobotContainer();
  frc2::Command* GetDefaultDriveCommand();
  frc2::Command* GetDefaultClimberCommand();
  frc2::Command* GetAutoCommand();
  frc2::Command* GetDoNothingCommand();
  Drivetrain* GetDrivetrain();
  Climber* GetClimber();
  private:
    frc::Joystick* m_joystick1;
    frc::Joystick* m_joystick2;
    frc2::JoystickButton* m_turboButton;
    frc::SendableChooser<frc2::Command*>* m_chooser;
    Drivetrain* m_drivetrain;
    Climber* m_climber;
    ArcadeDrive* m_defaultDriveCommand;
    ClimberMove* m_defaultClimberCommand;
    

    AutoTurnAngle* m_autoTurnCommand;
    AutoDriveForward* m_autoDriveCommand;
    DoNothing* m_doNothingCommand;
};
