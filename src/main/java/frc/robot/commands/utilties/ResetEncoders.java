package frc.robot.commands.utilties;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetEncoders extends CommandBase {
    public static void resetEncoders(){
        Drivetrain.frontLeft.setSelectedSensorPosition(0);
        Drivetrain.frontRight.setSelectedSensorPosition(0);
    }   
}