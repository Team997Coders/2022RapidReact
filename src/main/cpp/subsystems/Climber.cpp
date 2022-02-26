// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
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
    maximumPosition = minimumPosition + constants::Values::CLIMBER_UPPER_LIMIT;
}

void Climber::Set(double input) {
    input = std::clamp(input, -constants::Values::CLIMBER_MAX_SPEED, constants::Values::CLIMBER_MAX_SPEED);
    if (abs(input) <= constants::Values::CLIMBER_INPUT_DEADZONE) {
        input = 0;
    } 
    if (m_encoder -> GetPosition() + maximumPosition <= 0 && input <= 0) {
        input = 0;
    } 
    frc::SmartDashboard::PutNumber("climber encoder value", m_encoder -> GetPosition());
    frc::SmartDashboard::PutNumber("climber max pos", maximumPosition);
    m_climberMotor -> Set(input);
}

// This method will be called once per scheduler run
void Climber::Periodic() {}
