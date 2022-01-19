// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <thread>
#include <chrono>
#include "CustomAction.h"
#include <frc2/command/SubsystemBase.h>

// SpartanRunner 2.0
class CustomScheduler {
  public:
    CustomScheduler(int loopFrequencyMS);
    void Start();
    void Stop();
    void SetLoopFrequencyMS(int loopFrequencyMS);
    void AddAction(CustomAction action, bool shouldAutoDestruct);

  private:
    void Run();
    int loopFrequency;
    //long lastCycle;
    std::chrono::time_point<std::chrono::system_clock> lastCycle;
    bool halt;
    std::thread thr; 

};
