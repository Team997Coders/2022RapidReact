// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>

RobotContainer::RobotContainer() {
    m_joystick = new frc::Joystick(constants::Ports::CONTROLLER_1);

    m_drivetrain = new Drivetrain();

    m_defaultDriveCommand = new ArcadeDrive(m_drivetrain, 
        [this] { return -m_joystick->GetRawAxis(0); }, 
        [this] { return m_joystick->GetRawAxis(1); });
}

RobotContainer::~RobotContainer() {
    delete m_joystick;
    delete m_drivetrain;
    delete m_joystick;
}

void RobotContainer::Run() {
    frc2::CommandScheduler::GetInstance().Run();
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(m_drivetrain, *m_defaultDriveCommand);
}

void RobotContainer::Stop() {
    frc2::CommandScheduler::GetInstance().Disable();
}