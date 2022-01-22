// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include <frc/Joystick.h>

RobotContainer::RobotContainer() {
    m_joystick = new frc::Joystick(constants::Ports::CONTROLLER_1);

    m_drivetrain = new Drivetrain();

    m_defaultDriveCommand = new ArcadeDrive(m_drivetrain, 
        [m_joystick1 = m_joystick]() -> double { return -m_joystick1->GetRawAxis(0); }, 
        [m_joystick1 = m_joystick]() -> double { return m_joystick1->GetRawAxis(1); });
}


