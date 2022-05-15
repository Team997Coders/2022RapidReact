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

    /**
     * Creates the Climber subsystem with a single SparkMax NEO motor and a DIO
     * magnetic limit switch.
     */
    public Climber() {
        climberMotor = new CANSparkMax(Constants.Ports.CLIMBER_PORT, MotorType.kBrushless);
        climberMotor.restoreFactoryDefaults();
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.enableVoltageCompensation(Constants.MAX_VOLTAGE_DRAW_VOLTS);

        climberEncoder = climberMotor.getEncoder();

        climberZeroSwitch = new DigitalInput(Constants.Ports.ZERO_SWITCH_PORT);
    }

    public double getMotorCurrent() {
        return climberMotor.getOutputCurrent();
    }

    public boolean getZeroSwitch() {
        return !climberZeroSwitch.get();
    }

    /**
     * Gets the climber encoder's current position. While it is not an absolute
     * encoder, the climber starts the match and plays most of teleop at or very
     * close to its zero position and the magnetic switch that zeros the encoder,
     * meaning that there will not be much offset.
     * 
     * @return The relative position of the encoder, in rotations.
     */
    public double getEncoderPosition() {
        return climberEncoder.getPosition();
    }

    /**
     * Sets the speed of the climber motor.
     * 
     * @param movement : The value between [-1,1] (inclusive) to send to the motor
     *                 controller.
     * @param override : Whether to ignore soft-stop protections.
     */
    public void climberMove(double movement, boolean override) {
        if (getZeroSwitch()) {
            resetEncoder();
        }
        if ((Math.abs(movement) <= Constants.Controller.DEAD_BAND) ||
                (!override && climberEncoder.getPosition() >= Constants.Climber.CLIMBER_MAX_HEIGHT && movement < 0) ||
                (!override && getZeroSwitch() && movement > 0)) {
            movement = 0;
        }

        climberMotor.set(movement);
    }

    /**
     * Sets the encoder of the climber motor to zero.
     */
    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // the counter is in place to prevent constant sending of the zero switch's
        // status over NetworkTables, using some amount of the bandwith the USB camera
        // would like to have. Other SmartDashboard calls that don't have this feature
        // are in place because they are for debug, not actual match play.
        if (smartdashboardCounter >= 10) {
            SmartDashboard.putBoolean("Zero Switch", getZeroSwitch());
            smartdashboardCounter = 0;
        }
        smartdashboardCounter += 1;
    }
}
