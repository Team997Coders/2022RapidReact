// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
    public CANSparkMax climberMotor;
    public RelativeEncoder climberEncoder;
    private DigitalInput climberZeroSwitch;
    public Climber() {
        climberMotor = new CANSparkMax(Constants.Ports.CLIMBER_PORT, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberEncoder = climberMotor.getEncoder();
        climberZeroSwitch = new DigitalInput(Constants.Ports.ZERO_SWITCH_PORT);
        climberMotor.restoreFactoryDefaults();
        SmartDashboard.putBoolean("Zero Switch", climberZeroSwitch.get());
    }

    public void climberMove(double movement) {
        if (climberZeroSwitch.get()){
            climberEncoder.setPosition(0);
            if (movement < 0) {
                movement = 0;
            }
        }
        if (climberEncoder.getPosition() > Constants.MovementConstants.CLIMBER_MAX_HEIGHT && movement > 0) {
            movement = 0;
        }
        SmartDashboard.putNumber("final velocity", movement);
        climberMotor.set(movement);
    }

    @Override
    public void periodic() {
    }
}
