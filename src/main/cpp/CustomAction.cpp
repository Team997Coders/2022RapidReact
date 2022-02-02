// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomAction.h"
#include <frc2/command/Subsystem.h>

CustomAction::CustomAction(std::initializer_list<frc2::Subsystem*> requirements, CustomScheduler* scheduler)
: dependencies(requirements), scheduler(scheduler) {}

void CustomAction::Initialize() {

}

void CustomAction::Execute() {

}

void CustomAction::End(bool interrupted) {

}

bool CustomAction::IsFinished() {
    return true;
}

std::vector<frc2::Subsystem*> CustomAction::GetDependencies() {
    return dependencies;
}