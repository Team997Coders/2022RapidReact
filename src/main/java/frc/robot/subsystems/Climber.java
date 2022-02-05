package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public static CANSparkMax climberMotor;
    public static RelativeEncoder climberEncoder;
    private static DigitalInput climberZeroSwitch;
    public Climber() {
        climberMotor = new CANSparkMax(Constants.Ports.CLIMBER_PORT, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0);
        climberZeroSwitch = new DigitalInput(Constants.Ports.CLIMBER_ZERO_SWITCH_PORT);
    }
    public boolean zeroSwitchPressed() {
        return !climberZeroSwitch.get();
    }

    public void climberMove(double movement) {
        if (movement > 1) {
            movement = 1;
        }
        if (movement < -1) {
            movement = -1;
        }
        climberMotor.set(movement);
    }


    @Override
    public void periodic() {
        if (zeroSwitchPressed()) {
            climberEncoder.setPosition(0);
            climberMotor.set(0);
        }
        SmartDashboard.putNumber("Climber Motor Encoder", climberEncoder.getPosition());
        SmartDashboard.putBoolean("Zero Switch", zeroSwitchPressed());
    }
}