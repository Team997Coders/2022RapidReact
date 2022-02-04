// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <CustomScheduler.h>

class CustomAction {
  public:
    CustomAction(std::initializer_list<frc2::Subsystem*> requirements);
    virtual void Initialize();
    virtual void Execute();
    virtual void End(bool interrupted);
    virtual bool IsFinished();
    std::vector<frc2::Subsystem*> GetDependencies();
    bool hasInit = false;
    bool shouldEnd = false;
  private:
    std::vector<frc2::Subsystem*> dependencies;
};
