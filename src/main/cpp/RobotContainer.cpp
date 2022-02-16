// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
    m_joystick = new frc::Joystick(constants::Ports::CONTROLLER_1);

    m_drivetrain = new Drivetrain();
    
    m_defaultDriveCommand = new ArcadeDrive(m_drivetrain, 
        [this] { return -m_joystick->GetRawAxis(constants::Ports::DRIVE); }, 
        [this] { return m_joystick->GetRawAxis(constants::Ports::TURN); });

    m_defaultDriveAction = new ArcadeDriveAction(m_drivetrain, 
        [this] { return -m_joystick->GetRawAxis(constants::Ports::DRIVE); }, 
        [this] { return m_joystick->GetRawAxis(constants::Ports::TURN); });

    

    
}

RobotContainer::~RobotContainer() {
    delete m_joystick;
    delete m_drivetrain;
    delete m_joystick;
}

CustomAction* RobotContainer::GetDefaultDriveAction() { return m_defaultDriveAction; }
frc2::Command* RobotContainer::GetDefaultDriveCommand() { return m_defaultDriveCommand; }