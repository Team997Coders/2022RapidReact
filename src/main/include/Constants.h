// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/SPI.h>
#include <AHRSProtocol.h>

namespace constants {
  struct Ports {
    static const int
      FRONT_LEFT = 13, BACK_LEFT = 14, FRONT_RIGHT = 15, BACK_RIGHT = 16,
      CLIMBER = 9,
      CONTROLLER_1 = 0,
      DRIVE = 1, TURN = 4,
      CLIMBER_UP = 2, CLIMBER_DOWN = 3;
  };
  struct Values {
    static constexpr double 
      DRIVE_INPUT_MODIFIER = 0.25,
      TURN_INPUT_MODIFIER = 0.20,
      RAMPING_MODIFIER = 0.01;
  };
}
