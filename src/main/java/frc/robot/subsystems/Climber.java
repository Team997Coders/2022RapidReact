// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private CANSparkMax climberMotor;
    private RelativeEncoder climberEncoder;
    private DigitalInput climberZeroSwitch;
    private int smartdashboardCounter = 0;

    public Climber() {
        climberMotor = new CANSparkMax(Constants.Ports.CLIMBER_PORT, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake); // the climber needs to hang for a while- this prevents it from slipping
        climberEncoder = climberMotor.getEncoder();
        climberZeroSwitch = new DigitalInput(Constants.Ports.ZERO_SWITCH_PORT);
        climberMotor.restoreFactoryDefaults();
    }

    public double getMotorCurrent() {
        return climberMotor.getOutputCurrent();
    }
    
    public boolean getZeroSwitch() {
        return (!(climberZeroSwitch.get()));
    }
    
    public double getEncoderPosition() {
        return -climberEncoder.getPosition();
    }

    public void climberMove(double movement, boolean override) {
        if (getZeroSwitch())
        { 
            climberEncoder.setPosition(0);
        }
        if (
            (Math.abs(movement) <= Constants.Controller.DEAD_ZONE_SENSITIVITY) ||
            (!override && getEncoderPosition() >= Constants.Climber.CLIMBER_MAX_HEIGHT && movement < 0) ||
            (!override && getZeroSwitch() && movement > 0)
        ) { movement = 0; }

        climberMotor.set(movement);
    }

    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }
    
    @Override
    public void periodic() {
        if (smartdashboardCounter >= 10) {
            SmartDashboard.putBoolean("Zero Switch", getZeroSwitch());
            smartdashboardCounter = 0;
        }
        smartdashboardCounter += 1;
    }
}
