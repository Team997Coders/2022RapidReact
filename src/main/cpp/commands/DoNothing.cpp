// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DoNothing.h"
#include <frc/smartdashboard/SmartDashboard.h>

DoNothing::DoNothing(Drivetrain* drivetrain)
:m_drivetrain(drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

// Called when the command is initially scheduled.
void DoNothing::Initialize() {
  m_drivetrain -> ResetEncoders();
}

// Called repeatedly when this Command is scheduled to run
void DoNothing::Execute() {
  frc::SmartDashboard::PutNumber("ENCODER AVERAGE", m_drivetrain -> GetEncoderAverage());
  frc::SmartDashboard::PutNumber("LEFT ENCODER", m_drivetrain -> GetLeftEncoder());
  frc::SmartDashboard::PutNumber("RIGHT ENCODER", m_drivetrain -> GetRightEncoder());
}

// Called once the command ends or is interrupted.
void DoNothing::End(bool interrupted) {}

// Returns true when the command should end.
bool DoNothing::IsFinished() {
  return false;
}
