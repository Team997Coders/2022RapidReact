// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "actions/ClimberMove.h"
#include "subsystems/Climber.h"

ClimberMove::ClimberMove(Climber* climber, std::function<double()> up, std::function<double()> down)
 : CustomAction({climber}), m_climber(climber), m_up(up), m_down(down) {}

// Called when the command is initially scheduled.
void ClimberMove::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ClimberMove::Execute() {
  m_climber -> Set(m_up() - m_down());
}

// Called once the command ends or is interrupted.
void ClimberMove::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberMove::IsFinished() {
  return false;
}
