// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Drivetrain.h"
#include "Constants.h"

Drivetrain::Drivetrain() {
    frontLeft = new ctre::phoenix::motorcontrol::can::TalonFX(constants::Ports::FRONT_LEFT);
    backLeft = new ctre::phoenix::motorcontrol::can::TalonFX(constants::Ports::BACK_LEFT);
    frontRight = new ctre::phoenix::motorcontrol::can::TalonFX(constants::Ports::FRONT_RIGHT);
    backRight = new ctre::phoenix::motorcontrol::can::TalonFX(constants::Ports::BACK_RIGHT);

    backLeft -> Follow(*frontLeft, ctre::phoenix::motorcontrol::FollowerType::FollowerType_PercentOutput);
    backRight -> Follow(*frontRight, ctre::phoenix::motorcontrol::FollowerType::FollowerType_PercentOutput);

    frontLeft -> ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    frontRight -> ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);

    frontLeft -> SetInverted(true);
    backLeft -> SetInverted(true);

    gyro = new AHRS(frc::SPI::kMXP);
}

void Drivetrain::SetMotorOutput(double left, double right) {
    frontLeft -> Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, left);
    frontRight -> Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right);
}

void Drivetrain::SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode) {
    frontLeft -> SetNeutralMode(mode);
    backLeft -> SetNeutralMode(mode);
}

double Drivetrain::GetGyroAngle() { return gyro -> GetAngle(); }

Drivetrain::~Drivetrain() {
    delete frontLeft;
    delete frontRight;
    delete backLeft;
    delete backRight;

    delete gyro;
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}
