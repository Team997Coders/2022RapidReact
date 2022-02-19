// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoTurnAngle.h"

AutoTurnAngle::AutoTurnAngle(Drivetrain* drivetrain, double degrees) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

// Called when the command is initially scheduled.
void AutoTurnAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoTurnAngle::Execute() {}

// Called once the command ends or is interrupted.
void AutoTurnAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoTurnAngle::IsFinished() {
  return false;
}
