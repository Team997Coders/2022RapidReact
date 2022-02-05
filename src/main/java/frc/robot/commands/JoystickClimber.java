// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class JoystickClimber extends CommandBase {
  /** Creates a new JoystickClimber. */
  private Joystick js1;
  private Climber climber;

  public JoystickClimber(Climber climb) {
    addRequirements(climb);
    climber = climb;
    // Use addRequirements() here to declare subsystem dependencies. 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    js1 = new Joystick(Constants.Ports.CONTROLLER_1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("POV", js1.getPOV(0));
    int POV = js1.getPOV();
    if(POV == 0) {
      climber.Move(Constants.Ports.SPEED);
    }
    else if (POV == 180) {
      climber.Move(-1 * Constants.Ports.SPEED);
    }
    else {
      climber.Move(0);
    }
    //climber.Move(js1.getRawAxis(Constants.Ports.JOYSTICK_1));
    //climber.Move(js1.getPOV(Constants.Ports.JOYSTICK_1));
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
