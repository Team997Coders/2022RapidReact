// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <rev/CANSparkMax.h>
#include <algorithm>
#include "subsystems/Climber.h"
#include "Constants.h"

Climber::Climber() {
    m_climberMotor = new rev::CANSparkMax(constants::Ports::CLIMBER, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_climberMotor -> SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //m_encoder = new rev::SparkMaxRelativeEncoder()
    m_encoder = new rev::SparkMaxRelativeEncoder(m_climberMotor -> GetEncoder());  //m_climberMotor -> GetEncoder();
    minimumPosition = m_encoder -> GetPosition();
    maximumPosition = minimumPosition + maximumPosition;
}

void Climber::Set(double input) {
    input = std::clamp(input, -constants::Values::CLIMBER_MAX_SPEED, constants::Values::CLIMBER_MAX_SPEED);
    if (m_encoder -> GetPosition() - maximumPosition <= constants::Values::CLIMBER_ERROR_RANGE 
        || abs(input) >= constants::Values::CLIMBER_INPUT_DEADZONE /* || switch*/) {
        //input = 0;
    } 
    m_climberMotor -> Set(input);
}

// This method will be called once per scheduler run
void Climber::Periodic() {}
