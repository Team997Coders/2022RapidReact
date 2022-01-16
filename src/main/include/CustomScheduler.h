// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <thread>
#include "CustomAction.h"

class CustomScheduler {
  public:
    CustomScheduler(int loopFrequencyMS);
    void Run();
    void Stop();
    void SetLoopFrequencyMS(int loopFrequencyMS);
    void AddAction(CustomAction::CustomAction action, bool shouldAutoDestruct);

  private:
    int loopFrequency;
    long lastCycle;
    std::thread thr; 
};
