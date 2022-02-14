package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberPID extends CommandBase {
    private Climber climber;
    private Joystick joy;
    private double kP = 0.01; // replace with calls to constants once tuning complete
    private double kD = 0.0;
    private double kI = 0.0;

    public ClimberPID(Climber m_climber, Joystick joystick) { 
        addRequirements(m_climber);
        climber = m_climber;
        joy = joystick;
        reDisplayClimberPidGains();
    }
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.MovementConstants.ClimberConstants.CLIMB_CONSTRAINT_V, Constants.MovementConstants.ClimberConstants.CLIMB_CONSTRAINT_ACCEL);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kD, kI, m_constraints);
    private double targetHeight = 0;
    private double climberMotorDistance;
    private double PIDOutput;

    public void reDisplayClimberPidGains() {
        SmartDashboard.putNumber("climber kP", kP);
        SmartDashboard.putNumber("climber kI", kD);
        SmartDashboard.putNumber("climber kD", kI);
    }

    @Override
    public void initialize() {
        kP = SmartDashboard.getNumber("climber kP", 0); // intentionally like this: if network tables not returning anything, don't move
        kI = SmartDashboard.getNumber("climber kI", 0);
        kD = SmartDashboard.getNumber("climber kD", 0);

        m_controller.setPID(kP, kI, kD);
    }

    @Override
    public void execute() {

        if (-1*joy.getRawAxis(Constants.Controller.JOYSTICK_1) > 0.1) {
            targetHeight += 0.1;
        }
        else if (-1*joy.getRawAxis(Constants.Controller.JOYSTICK_1) < -0.1) {
            targetHeight -= 0.1;
        }
        m_controller.setGoal(targetHeight);
        climberMotorDistance = climber.climberEncoder.getPosition();
        PIDOutput = m_controller.calculate(climberMotorDistance);
        climber.climberMove(PIDOutput);
        
    
        SmartDashboard.putNumber("climber error", m_controller.getSetpoint().position-climberMotorDistance);
        SmartDashboard.putNumber("climber targetHeight", targetHeight);
        SmartDashboard.putNumber("climber MotorDistance", climberMotorDistance);
        SmartDashboard.putNumber("climber PIDOutput", PIDOutput);
        SmartDashboard.putNumber("climber setPoint", m_controller.getSetpoint().position);
        SmartDashboard.putNumber("climber setPoint velocity", m_controller.getSetpoint().velocity);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished() {
        return false;
    }
}