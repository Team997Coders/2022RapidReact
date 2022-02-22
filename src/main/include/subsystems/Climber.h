// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc2/command/SubsystemBase.h>

class Climber : public frc2::SubsystemBase {
  public:
    Climber();
    void Set(double input);
    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void Periodic() override;

  private:
    rev::CANSparkMax* m_climberMotor; 
    rev::SparkMaxRelativeEncoder* m_encoder;
    double minimumPosition;
    double maximumPosition;
};
