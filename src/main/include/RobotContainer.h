// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include "subsystems/Drivetrain.h"

class RobotContainer {
 public:
  RobotContainer();

  private:
    frc::Joystick* m_joystick;
    Drivetrain* m_drivetrain;

};
