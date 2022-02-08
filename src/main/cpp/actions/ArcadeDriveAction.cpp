// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "actions/ArcadeDriveAction.h"
#include "subsystems/Drivetrain.h"

ArcadeDriveAction::ArcadeDriveAction(Drivetrain* drivetrain, std::function<double()> x, std::function<double()> z)
 : CustomAction({drivetrain}), m_drivetrain(drivetrain), m_x(x), m_z(z) {}

// Called when the command is initially scheduled.
void ArcadeDriveAction::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ArcadeDriveAction::Execute() {
  m_drivetrain -> SetMotorOutput(m_x() + m_z(), m_x() - m_z());
}

// Called once the command ends or is interrupted.
void ArcadeDriveAction::End(bool interrupted) {}

// Returns true when the command should end.
bool ArcadeDriveAction::IsFinished() {
  return false;
}
