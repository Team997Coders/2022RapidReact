// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomScheduler.h"
#include "CustomAction.h"
#include <chrono>
#include <mutex>
#include <future>
#include <frc2/command/Subsystem.h>

CustomScheduler::CustomScheduler(int loopFrequencyMS) : runningLoopFrequency(loopFrequencyMS), schedulingLoopFrequency(loopFrequencyMS) {

}

void CustomScheduler::Run() {
    long lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    while (!halt) {
        if (lastCycle + runningLoopFrequency < std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count()) {
            runningActionsLock.lock();
            lastCycle = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            for (std::list<CustomAction*>::iterator it = runningActions.begin(); it != runningActions.end(); ++it) {
                if (!(*it) -> hasInit) {
                    (*it) -> Initialize();
                }
                if ((*it) -> IsFinished()) {
                    (*it) -> shouldEnd = true;
                } else {
                    (*it) -> Execute();
                }
                if ((*it) -> shouldEnd) {
                    (*it) -> End(false);
                    runningActions.remove(*it);
                }
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
            for (std::list<CustomAction*>::iterator it1 = scheduledActions.begin(); it1 != scheduledActions.end(); ++it1) {
                bool canBeScheduled = true;
                for (std::vector<frc2::Subsystem*>::iterator it2 = (*it1) -> GetDependencies().begin(); it2 != (*it1) -> GetDependencies().end(); ++it2) {
                    if (!accessibleResources[*it2]) {
                        canBeScheduled = false;
                    }
                }
                if (canBeScheduled) {
                    runningActionsLock.lock();
                    runningActions.push_back(*it1);
                    scheduledActions.remove(*it1);
                    runningActionsLock.unlock();
                }
            }
            scheduledActionsLock.lock();
        }
        
    }
}

void CustomScheduler::RunAction(CustomAction* action, bool shouldDestruct) {
    scheduledActionsLock.lock();
    scheduledActions.push_back(action);
    // for each key in if resource key does not exist in accessible resources, add one and set to true
    for (std::vector<frc2::Subsystem*>::iterator it = action -> GetDependencies().begin(); it != action -> GetDependencies().end(); ++it) {
        if (accessibleResources.find(*it) != accessibleResources.end()) {
            accessibleResources[*it] = true;
        }
    }
    scheduledActionsLock.unlock();
}

void CustomScheduler::Start() {
    halt = false;
    std::async(std::launch::async, Run, this);
    std::async(std::launch::async, Schedule, this);
}

void CustomScheduler::Stop() {
    halt = true;
}
