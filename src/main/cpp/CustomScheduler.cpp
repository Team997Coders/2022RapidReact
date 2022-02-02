// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomScheduler.h"
#include <chrono>
#include <mutex>

CustomScheduler::CustomScheduler(int loopFrequencyMS) : runningLoopFrequency(loopFrequencyMS), schedulingLoopFrequency(loopFrequencyMS) {
    
}

void CustomScheduler::Run() {
    long lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    while (!halt) {
        if (lastCycle + runningLoopFrequency < std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count()) {
            runningActionsLock.lock();
            lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            for (int i = runningActions.size(); i >= 0; i--) {

            }
            runningActionsLock.unlock();
        }
    }
}

void CustomScheduler::Schedule() {
    long lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    while (!halt) {
        if (lastCycle + runningLoopFrequency < std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count()) {
            scheduledActionsLock.lock();
            lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            for (std::list<CustomAction*>::iterator it = scheduledActions.begin(); it != scheduledActions.end(); ++it) {
                if ((*it) -> GetDependencies())
            }
            scheduledActionsLock.lock();
        }
        
    }
}

void CustomScheduler::RunAction(CustomAction* action, bool shouldDestruct) {
    scheduledActionsLock.lock();
    scheduledActions.push_back(action);
    scheduledActionsLock.unlock();
}

void CustomScheduler::Start() {
    halt = false;
}

void CustomScheduler::Stop() {
    halt = true;
}
