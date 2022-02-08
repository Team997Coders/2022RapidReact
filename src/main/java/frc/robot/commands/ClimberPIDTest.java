package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberPIDTest extends CommandBase {
    private Climber climber;
    public ClimberPIDTest(Climber m_climber) { 
        addRequirements(m_climber);
        climber = m_climber;
    }
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(0.1, 0.0, 0.0, m_constraints);
    private double targetHeight = 0;
    private double climberMotorDistance = 0;
    private double PIDOutput = 0;

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (-1*RobotContainer.js1.getRawAxis(Constants.Controller.JOYSTICK_1) > 0.1) {
            targetHeight += 0.1;
        }
        else if (-1*RobotContainer.js1.getRawAxis(Constants.Controller.JOYSTICK_1) < -0.1) {
            targetHeight -= 0.1;
        }
        climberMotorDistance = Climber.climberEncoder.getPosition()*2*Math.PI*Constants.MovementConstants.NEO_ENCODER_RADIUS;
        m_controller.setGoal(targetHeight);
        PIDOutput = m_controller.calculate(climberMotorDistance);
        climber.climberMove(PIDOutput);
        SmartDashboard.putNumber("targetHeight", targetHeight);
        SmartDashboard.putNumber("climberMotorDistance", climberMotorDistance);
        SmartDashboard.putNumber("PIDOutput", PIDOutput);
        SmartDashboard.putNumber("setPoint", m_controller.getSetpoint().position);
        SmartDashboard.putNumber("setPoint velocity", m_controller.getSetpoint().velocity);

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