// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <AHRS.h>
#include <frc2/command/SubsystemBase.h>

class Drivetrain : public frc2::SubsystemBase {
  public:
    Drivetrain();
    ~Drivetrain();

    void SetMotorOutput(double left, double right);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
    void Periodic() override;
    void SetMotors(double x, double z);

  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    ctre::phoenix::motorcontrol::can::TalonFX* frontRight;
    ctre::phoenix::motorcontrol::can::TalonFX* backRight;
    ctre::phoenix::motorcontrol::can::TalonFX* frontLeft;
    ctre::phoenix::motorcontrol::can::TalonFX* backLeft;

    AHRS* gyro;
};
