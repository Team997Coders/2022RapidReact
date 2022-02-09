// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ClimberPIDTest;
//import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.JoystickClimber;
import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Joystick js1;
  public JoystickButton resetPidGainsButton;
  private Climber m_climber;
  private ClimberPIDTest m_climberPID;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    js1 = new Joystick(Constants.Controller.JOYSTICK_1);
    m_climber = new Climber();
    // private JoystickClimber m_joystickClimber = new JoystickClimber(m_climber, js1);
    m_climberPID = new ClimberPIDTest(m_climber, js1);
    // private ArcadeDrive m_arcadedrive = new ArcadeDrive(m_drive, js1);
    // Configure the button bindings
    configureButtonBindings();
  }

  public void setDefaultCommands() {
    //CommandScheduler.getInstance().setDefaultCommand(m_drive, m_arcadedrive);
    // CommandScheduler.getInstance().setDefaultCommand(m_climber, m_joystickClimber);
    CommandScheduler.getInstance().setDefaultCommand(m_climber, m_climberPID);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    resetPidGainsButton = new JoystickButton(js1, Constants.Controller.A_BUTTON);

    resetPidGainsButton.whenPressed(m_climberPID::reDisplayPidGains);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(() -> m_climber.climberMove(0.4));
    }
}
