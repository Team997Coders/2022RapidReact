package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberPIDTest extends CommandBase {
    private Climber climber;
    private Joystick joy;
    private double kP = 0.0;
    private double kD = 0.0;
    private double kI = 0.0;

    public ClimberPIDTest(Climber m_climber, Joystick joystick) { 
        addRequirements(m_climber);
        climber = m_climber;
        joy = joystick;
    }
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(0.01, 0.0, 0.0, m_constraints);
    private double targetHeight = 0;
    private double climberMotorDistance;
    private double PIDOutput;

    public void reDisplayPidGains() {
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        kP = SmartDashboard.getNumber("kP", 0);
        kI = SmartDashboard.getNumber("kI", 0);
        kD = SmartDashboard.getNumber("kD", 0);

        m_controller.setPID(kP, kI, kD);

        if (-1*joy.getRawAxis(Constants.Controller.JOYSTICK_1) > 0.1) {
            targetHeight += 0.1;
        }
        else if (-1*joy.getRawAxis(Constants.Controller.JOYSTICK_1) < -0.1) {
            targetHeight -= 0.1;
        }
        // climberMotorDistance = climber.climberEncoder.getPosition()*2*Math.PI*Constants.MovementConstants.NEO_ENCODER_RADIUS;
        m_controller.setGoal(targetHeight);
        climberMotorDistance = climber.climberEncoder.getPosition();
        PIDOutput = m_controller.calculate(climberMotorDistance);
        climber.climberMove(PIDOutput);
        
    

        SmartDashboard.putNumber("targetHeight", targetHeight);
        SmartDashboard.putNumber("climberMotorDistance", climberMotorDistance);
        SmartDashboard.putNumber("PIDOutput", PIDOutput);
        SmartDashboard.putNumber("setPoint", m_controller.getSetpoint().position);
        SmartDashboard.putNumber("setPoint velocity", m_controller.getSetpoint().velocity);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished() {
        return false;
    }
}