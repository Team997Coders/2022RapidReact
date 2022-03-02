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
    }

    public void climberMove(double movement) {
        if (!climberZeroSwitch.get()){
            climberEncoder.setPosition(0);
            if (movement > 0) {
                movement = 0;
            }
        }
        if (-climberEncoder.getPosition() > Constants.CLIMBER_MAX_HEIGHT && movement < 0) {
            movement = 0;
        }
        climberMotor.set(movement);
    }
    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Zero Switch", !climberZeroSwitch.get());
        SmartDashboard.putNumber("Delta Climber Encoder", climberEncoder.getVelocity());
        SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
        SmartDashboard.putNumber("NavX Pitch", Drivetrain.gyro.getPitch());
        SmartDashboard.putNumber("NavX Yaw", Drivetrain.gyro.getYaw());
        SmartDashboard.putNumber("NavX Roll", Drivetrain.gyro.getRoll());
        SmartDashboard.putNumber("Delta NavX Roll", Drivetrain.gyro.getRawGyroX());
        SmartDashboard.putNumber("Delta NavX Pitch", Drivetrain.gyro.getRawGyroY());
        SmartDashboard.putNumber("Delta NavX Yaw", Drivetrain.gyro.getRawGyroZ());
    }
}