// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomScheduler.h"
#include <chrono>

CustomScheduler::CustomScheduler(int loopFrequencyMS) : loopFrequency(loopFrequencyMS) {
    
}

void CustomScheduler::Run() {
    lastCycle = std::chrono::system_clock::now();
    while (!halt) {

    }
}

void CustomScheduler::Start() {
    halt = false;
}

void CustomScheduler::Stop() {
    halt = true;
    thr.join();
}
