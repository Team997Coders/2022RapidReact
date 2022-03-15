// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/length.h>

#include "subsystems/Drivetrain.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoDriveForward
    : public frc2::CommandHelper<frc2::CommandBase, AutoDriveForward> {
  public:
    AutoDriveForward(Drivetrain* drivetrain, double distance);
    //~AutoDriveForward();
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
  private:
    double m_distance;
    double targetPosition;
    double initialPosition;
    double output;
    double lastOutput;
    Drivetrain* m_drivetrain;
    frc::ProfiledPIDController<units::feet>* pidController;
};