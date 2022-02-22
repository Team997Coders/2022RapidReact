// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include <algorithm>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "commands/ArcadeDrive.h"
#include "subsystems/Drivetrain.h"

ArcadeDrive::ArcadeDrive(Drivetrain* drivetrain, std::function<double()> x, std::function<double()> z, std::function<bool()> turbo) 
: m_drivetrain(drivetrain), m_x(x), m_z(z), m_turbo(turbo)
{
   AddRequirements(drivetrain);
 }

// Called when the command is initially scheduled.
void ArcadeDrive::Initialize() {
  m_drivetrain -> SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  lastLeft = 0;
  lastRight = 0;
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {
  double left = ((m_x() * constants::Values::DRIVE_INPUT_MODIFIER) + m_z() * constants::Values::TURN_INPUT_MODIFIER);
  double right = ((m_x() * constants::Values::DRIVE_INPUT_MODIFIER) - m_z() * constants::Values::TURN_INPUT_MODIFIER);
  
  if (m_turbo()) {
    left *= constants::Values::TURBO_MODIFIER;
    right *= constants::Values::TURBO_MODIFIER;
  }

  /*
  if (left - lastLeft >= constants::Values::RAMPING_MODIFIER) {
    left = lastLeft + constants::Values::RAMPING_MODIFIER;
  } else if (left - lastLeft <= -constants::Values::RAMPING_MODIFIER) {
    left = lastLeft - constants::Values::RAMPING_MODIFIER;
  }

  if (right - lastRight >= constants::Values::RAMPING_MODIFIER) {
    right = lastRight + constants::Values::RAMPING_MODIFIER;
  } else if (right - lastRight <= -constants::Values::RAMPING_MODIFIER) {
    right = lastRight - constants::Values::RAMPING_MODIFIER;
  }
*/
  lastLeft = std::clamp(left, lastLeft - constants::Values::RAMPING_MODIFIER, lastLeft + constants::Values::RAMPING_MODIFIER);
  lastRight = std::clamp(right, lastRight - constants::Values::RAMPING_MODIFIER, lastRight + constants::Values::RAMPING_MODIFIER);

  m_drivetrain -> SetMotorOutput(lastLeft, lastRight);
}

// Called once the command ends or is interrupted.
void ArcadeDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool ArcadeDrive::IsFinished() {
  return false;
}
