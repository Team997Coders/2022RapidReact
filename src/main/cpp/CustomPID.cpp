// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomPID.h"
#include <chrono>
#include <algorithm>

CustomPID::CustomPID(double p, double i, double d, double maxVelocity, double maxAcceleration)
: m_p(p), m_i(i), m_d(d), maxVel(maxVelocity), maxAccel(maxAcceleration), lastError(NULL), lastTime(NULL), lastOutput(NULL) {}

double CustomPID::Calculate(double currentDist, double targetDist) {
    double error = targetDist - currentDist;
    if (lastError == NULL) { lastError = error; }
    if (lastTime == NULL) { lastTime = clock(); }
    if (lastOutput == NULL) { lastOutput = 0; }

    double deltaT = GetDeltaT(lastTime);
    double derivative = (error - lastError) / deltaT;
    accumulation += error * deltaT;

    lastError = error;
    lastOutput = std::clamp(m_p * error + m_d * derivative + m_i * accumulation,
        std::max(lastOutput - maxAccel, maxVel),
        std::min(lastOutput + maxAccel, maxVel));
    return lastOutput;
}

double CustomPID::GetDeltaT(clock_t last) {
    return ((double) clock() - last) / CLOCKS_PER_SEC;
}