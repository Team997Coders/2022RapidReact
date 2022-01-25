// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "commands/ArcadeDrive.h"
#include "subsystems/Drivetrain.h"

//ArcadeDrive::ArcadeDrive(Drivetrain* drivetrain, std::function<double(void)> x, std::function<double(void)> z)
// : CustomAction({drivetrain}), m_drivetrain(drivetrain), m_x(x), m_z(z) {
ArcadeDrive::ArcadeDrive(Drivetrain* drivetrain, frc::Joystick* js) 
: m_drivetrain(drivetrain), m_js(js)
{
   AddRequirements(drivetrain);
 }

// Called when the command is initially scheduled.
void ArcadeDrive::Initialize() {
  frc::SmartDashboard::PutString("This3", "initialized command");  
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {
  //frc::SmartDashboard::PutNumber("forward output", m_x());
  //m_drivetrain -> SetMotorOutput(m_x() + m_z(), m_x() - m_z());
  frc::SmartDashboard::PutString("This5", "started executing");  
  m_drivetrain -> SetMotorOutput(m_js -> GetRawAxis(1) + m_js -> GetRawAxis(4), m_js -> GetRawAxis(1) - m_js -> GetRawAxis(4));
  frc::SmartDashboard::PutString("This4", "executed");  
}

// Called once the command ends or is interrupted.
void ArcadeDrive::End(bool interrupted) {
  frc::SmartDashboard::PutString("ended", "True");  
}

// Returns true when the command should end.
bool ArcadeDrive::IsFinished() {
  return false;
}
