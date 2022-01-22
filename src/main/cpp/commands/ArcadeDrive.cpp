// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArcadeDrive.h"
#include "subsystems/Drivetrain.h"

ArcadeDrive::ArcadeDrive(Drivetrain* drivetrain, std::function<double(void)> x, std::function<double(void)> z)
 : CustomAction({drivetrain}), m_drivetrain(drivetrain), m_x(x), m_z(z) {}

// Called when the command is initially scheduled.
void ArcadeDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {
  m_drivetrain -> SetMotorOutput(m_x() + m_z(), m_z() - m_z());
}

// Called once the command ends or is interrupted.
void ArcadeDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool ArcadeDrive::IsFinished() {
  return false;
}
