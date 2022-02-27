// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/SPI.h>
#include <AHRSProtocol.h>

namespace constants {
  struct Ports {
    static const int
      // motor controllers
      FRONT_LEFT = 13, BACK_LEFT = 14, FRONT_RIGHT = 15, BACK_RIGHT = 16,
      CLIMBER = 8,
      // joystick
      CONTROLLER_1 = 0,
      DRIVE = 1, TURN = 4,
      DRIVE_TURBO = 5,
      CLIMBER_UP = 2, CLIMBER_DOWN = 3;
  };
  struct Values {
    static constexpr double 
      // drive modifiers
      DRIVE_INPUT_MODIFIER = 0.25,
      TURN_INPUT_MODIFIER = 0.20,
      RAMPING_MODIFIER = 0.01,
      TURBO_MODIFIER = 2,

      // climber
      CLIMBER_INPUT_DEADZONE = 0.05,
      CLIMBER_MAX_SPEED = 0.3,
      CLIMBER_INPUT_MODIFIER = 0.2,
      CLIMBER_UPPER_LIMIT = 139,

      // PID
      DRIVE_P = 0.0,
      DRIVE_I = 0.0,
      DRIVE_D = 0.0,

      TURN_P = 0.01,
      TURN_I = 0.0,
      TURN_D = 0.0,

      CLIMB_UP_P = 0.0,
      CLIMB_UP_I = 0.0,
      CLIMB_UP_D = 0.0,

      CLIMB_DOWN_P = 0.0,
      CLIMB_DOWN_I = 0.0,
      CLIMB_DOWN_D = 0.0;
  };
}
