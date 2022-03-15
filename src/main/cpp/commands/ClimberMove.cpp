// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberMove.h"

ClimberMove::ClimberMove(Climber* climber, std::function<double()> up, std::function<double()> down, std::function<bool()> unlock)
: m_climber(climber), m_up(up), m_down(down), m_unlock(unlock) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void ClimberMove::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberMove::Execute() {
  m_climber -> Set(m_up() - m_down(), m_unlock());
}

// Called once the command ends or is interrupted.
void ClimberMove::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberMove::IsFinished() {
  return false;
}