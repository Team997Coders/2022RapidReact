// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <rev/CANSparkMax.h>

#include "subsystems/Climber.h"
#include "Constants.h"

Climber::Climber() {
    m_climberMotor = new rev::CANSparkMax(constants::Ports::CLIMBER, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_relativeEncoder = &(m_climberMotor -> GetEncoder());
    
    m_climberMotor -> SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    minimumPosition = m_relativeEncoder -> GetPosition();
    maximumPosition = minimumPosition + maximumPosition;
}

void Climber::Set(double input) {
    if (input >= constants::Values::CLIMBER_MAX_SPEED) {
        input = constants::Values::CLIMBER_MAX_SPEED;
    } else if (input <= -constants::Values::CLIMBER_MAX_SPEED) {
        input = -constants::Values::CLIMBER_MAX_SPEED;
    }
    if (m_relativeEncoder -> GetPosition() - maximumPosition <= constants::Values::CLIMBER_ERROR_RANGE/* || switch*/) {
        input = 0;
    } 
    m_climberMotor -> Set(input);
}

// This method will be called once per scheduler run
void Climber::Periodic() {}
