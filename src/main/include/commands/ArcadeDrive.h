// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include <subsystems/Drivetrain.h>
#include "CustomAction.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

// CustomAction may require modification
//class ArcadeDrive : public CustomAction {
class ArcadeDrive : public frc2::CommandHelper<frc2::CommandBase, ArcadeDrive> {
  public:
    ArcadeDrive(Drivetrain* drivetrain, frc::Joystick* js);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
  private:
    Drivetrain* m_drivetrain;
    frc::Joystick* m_js;
    //std::function<double()> m_x;
    //std::function<double()> m_z;
};
