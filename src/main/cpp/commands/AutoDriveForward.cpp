// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveForward.h"

AutoDriveForward::AutoDriveForward(Drivetrain* drivetrain, double distance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

// Called when the command is initially scheduled.
void AutoDriveForward::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoDriveForward::Execute() {}

// Called once the command ends or is interrupted.
void AutoDriveForward::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDriveForward::IsFinished() {
  return false;
}
