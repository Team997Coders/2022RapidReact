// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveForward.h"
#include "Constants.h"
#include <units/acceleration.h>
#include <units/velocity.h>
#include <wpi/numbers>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>

AutoDriveForward::AutoDriveForward(Drivetrain* drivetrain, double distance)
: m_drivetrain(drivetrain), m_distance(distance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
  pidController = new frc::ProfiledPIDController<units::feet>(constants::Values::DRIVE_P, constants::Values::DRIVE_I,
  constants::Values::DRIVE_D, frc::TrapezoidProfile<units::feet>::Constraints{10_fps, 1_fps_sq});
}

// Called when the command is initially scheduled.
void AutoDriveForward::Initialize() {
  m_drivetrain -> ResetEncoders();
  initialPosition = (m_drivetrain -> GetLeftEncoder() + m_drivetrain -> GetRightEncoder()) / 2;
  targetPosition = initialPosition + m_distance;
  output = 0;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveForward::Execute() {
  lastOutput = output;
  output = pidController -> Calculate(units::foot_t(((m_drivetrain -> GetLeftEncoder() + m_drivetrain -> GetRightEncoder()) / 2) * constants::Values::TICKS_TO_FEET), units::foot_t(targetPosition));
  m_drivetrain -> SetMotorOutput(output, output);
}

// Called once the command ends or is interrupted.
void AutoDriveForward::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDriveForward::IsFinished() {
  if (abs(m_distance - targetPosition) <= constants::Values::AUTO_DRIVE_ERROR_TOLERANCE
   && abs(lastOutput - output) <= constants::Values::AUTO_DRIVE_STOPPING_ACCELERATION) return true;
  return false;
}
