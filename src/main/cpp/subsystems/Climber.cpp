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
    m_sensor = new frc::DigitalInput(constants::Ports::CLIMBER_SENSOR);
    m_encoder = new rev::SparkMaxRelativeEncoder(m_climberMotor -> GetEncoder());
    isSet = false;
}

void Climber::Set(double input, bool override) {
    input = -std::clamp(input, -constants::Values::CLIMBER_MAX_SPEED, constants::Values::CLIMBER_MAX_SPEED);
    frc::SmartDashboard::PutBoolean("CLIMBER SENSOR", !(m_sensor -> Get()));
    if (!(m_sensor -> Get())) {
        maximumPosition = -(m_encoder -> GetPosition()) + constants::Values::CLIMBER_UPPER_LIMIT;
        isSet = true;
    }
    if (!override && ((abs(input) <= constants::Values::CLIMBER_INPUT_DEADZONE) || 
    (((-(m_encoder -> GetPosition()) - maximumPosition < 0 || !isSet) && input <= 0)) ||
    (!(m_sensor -> Get()) && input >= 0))) {
        input = 0;
    } 
    frc::SmartDashboard::PutNumber("climber encoder value", m_encoder -> GetPosition());
    frc::SmartDashboard::PutNumber("climber max pos", maximumPosition);
    m_climberMotor -> Set(input);
}

Climber::~Climber() {
    delete m_climberMotor;
    delete m_encoder;
}

// This method will be called once per scheduler run
void Climber::Periodic() {}
