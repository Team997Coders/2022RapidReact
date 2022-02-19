// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include "commands/AutoClimb.h"

AutoClimb::AutoClimb(Climber* climber) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void AutoClimb::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoClimb::Execute() {}

// Called once the command ends or is interrupted.
void AutoClimb::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoClimb::IsFinished() {
  return false;
}
