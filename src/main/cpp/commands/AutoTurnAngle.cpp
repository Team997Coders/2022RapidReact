// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include "commands/AutoTurnAngle.h"
#include <wpi/numbers>
#include "Constants.h"

AutoTurnAngle::AutoTurnAngle(Drivetrain* drivetrain, double degrees)
: m_drivetrain(drivetrain), m_degrees(degrees) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
  pidController = new frc::ProfiledPIDController<units::degrees>(constants::Values::TURN_P, constants::Values::TURN_I,
  constants::Values::TURN_D, frc::TrapezoidProfile<units::degrees>::Constraints{100_deg_per_s, 30_deg_per_s_sq});
}

// Called when the command is initially scheduled.
void AutoTurnAngle::Initialize() {
  frc::SmartDashboard::PutBoolean("INITIALIZE RAN", true);
  initialAngle = m_drivetrain -> GetGyroAngle();
  targetAngle = initialAngle + m_degrees;
}

// Called repeatedly when this Command is scheduled to run
void AutoTurnAngle::Execute() {
  // MAKE SURE THIS TYPE CASTING ACTUALLY WORKS
  frc::SmartDashboard::PutNumber("TURN ANGLE", m_drivetrain -> GetGyroAngle());
  double output = pidController -> Calculate((units::angle::degree_t) (m_drivetrain -> GetGyroAngle()), (units::angle::degree_t) targetAngle);
  m_drivetrain -> SetMotorOutput(-output, output);
}

// Called once the command ends or is interrupted.
void AutoTurnAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoTurnAngle::IsFinished() {
  return false;
}

AutoTurnAngle::~AutoTurnAngle() {
  delete pidController;
}