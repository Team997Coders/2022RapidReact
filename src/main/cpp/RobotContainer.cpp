// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
    frc::SmartDashboard::PutBoolean("STUFF EXISTS", true);
    m_joystick1 = new frc::Joystick(constants::Ports::CONTROLLER_1);

    m_drivetrain = new Drivetrain();
    m_climber = new Climber();
    
    m_defaultDriveCommand = new ArcadeDrive(
        m_drivetrain, 
        [this] { return -m_joystick1 -> GetRawAxis(constants::Ports::DRIVE); }, 
        [this] { return -m_joystick1 -> GetRawAxis(constants::Ports::TURN); },
        [this] { return m_joystick1 -> GetRawButton(constants::Ports::DRIVE_TURBO); }
    );

    m_defaultClimberCommand = new ClimberMove(
        m_climber,
        [this] { return m_joystick1 -> GetRawAxis(constants::Ports::CLIMBER_UP); },
        [this] { return m_joystick1 -> GetRawAxis(constants::Ports::CLIMBER_DOWN); }
    );

    m_autoTurnCommand = new AutoTurnAngle(m_drivetrain, 120);
    m_autoDriveCommand = new AutoDriveForward(m_drivetrain, 3);
    
    m_climber -> SetDefaultCommand(*m_defaultClimberCommand);
    m_drivetrain -> SetDefaultCommand(*m_defaultDriveCommand);
}

RobotContainer::~RobotContainer() {
    delete m_joystick1;
    delete m_drivetrain;
    delete m_climber;
    delete m_defaultDriveCommand;
    delete m_defaultClimberCommand;
    delete m_autoTurnCommand;
    delete m_autoDriveCommand;
}

frc2::Command* RobotContainer::GetDefaultDriveCommand() { return m_defaultDriveCommand; }
frc2::Command* RobotContainer::GetDefaultClimberCommand() { return m_defaultClimberCommand; }
frc2::Command* RobotContainer::GetAutoCommand() { return m_autoDriveCommand; }

Drivetrain* RobotContainer::GetDrivetrain() { return m_drivetrain; }
Climber* RobotContainer::GetClimber() { return m_climber; }