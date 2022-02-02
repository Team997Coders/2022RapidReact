// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomScheduler.h"
#include <chrono>

CustomScheduler::CustomScheduler(int loopFrequencyMS) : runningLoopFrequency(loopFrequencyMS), schedulingLoopFrequency(loopFrequencyMS) {
    
}

void CustomScheduler::Run() {
    long lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    while (!halt) {
        if (lastCycle + runningLoopFrequency < std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count()) {
            lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            for (int i = runningActions.size(); i >= 0; i--) {

            }
        }
    }
}

void CustomScheduler::Schedule() {
    while (!halt) {
        for (int i = scheduledActions.size(); i >= 0; i--) {

        }
    }
}

void CustomScheduler::Start() {
    halt = false;
}

void CustomScheduler::Stop() {
    halt = true;
    schedulerThread.join(); //comment this if you don't want to wait for thread
}
