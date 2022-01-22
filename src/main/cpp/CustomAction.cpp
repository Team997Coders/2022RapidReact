// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomAction.h"
#include <frc2/command/Subsystem.h>

CustomAction::CustomAction(std::initializer_list<frc2::Subsystem*> requirements) {
    dependencies = requirements;
}