// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <mutex>
#include <chrono>
#include <list>
#include <map>
#include <thread>
#include "CustomAction.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>

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
    std::map<frc2::Subsystem*, bool> accessibleResources; //maybe use unordered_map for faster access
    std::thread thr1;
    std::thread thr2;
};
