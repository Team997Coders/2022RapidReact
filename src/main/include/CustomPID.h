// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>

class CustomPID {
  public:
    CustomPID(double p, double i, double d, double maxVelocity, double maxAcceleration);
    double Calculate(double currentDist, double targetDist);
  private:
    double maxAccel;
    double maxVel;
    double m_p;
    double m_i;
    double m_d;
    double accumulation;
    double lastError;
    double lastOutput;
    clock_t lastTime;
    double GetDeltaT(clock_t last);
};
