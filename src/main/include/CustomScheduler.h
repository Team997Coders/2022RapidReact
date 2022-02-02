// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <mutex>
#include <chrono>
#include <list>
#include "CustomAction.h"
#include <frc2/command/SubsystemBase.h>

// SpartanRunner 2.0
class CustomScheduler {
  public:
    CustomScheduler(int loopFrequencyMS);
    void Start();
    void Stop();
    void SetLoopFrequencyMS(int loopFrequencyMS);
    void RunAction(CustomAction* action, bool shouldDestruct);
  private:
    void Run();
    void Schedule();
    int runningLoopFrequency;
    int schedulingLoopFrequency;
    bool halt;
    std::list<CustomAction*> scheduledActions;
    std::list<CustomAction*> runningActions;
    std::mutex scheduledActionsLock;
    std::mutex runningActionsLock;

};
