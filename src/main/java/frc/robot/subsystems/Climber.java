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
    public CANSparkMax climberMotor;
    public RelativeEncoder climberEncoder;
    private DigitalInput climberZeroSwitch;
    public Climber() {
        climberMotor = new CANSparkMax(Constants.Ports.CLIMBER_PORT, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberEncoder = climberMotor.getEncoder();
        //climberEncoder.setPosition(0);
        climberZeroSwitch = new DigitalInput(Constants.Ports.CLIMBER_ZERO_SWITCH_PORT);
        climberMotor.restoreFactoryDefaults();
        SmartDashboard.putBoolean("Zero Switch", zeroSwitchPressed());
    }
    public boolean zeroSwitchPressed() {
        return !climberZeroSwitch.get();
    }

    public void climberMove(double movement) {
        if (zeroSwitchPressed()){
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