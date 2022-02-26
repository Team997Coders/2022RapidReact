// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Pathweaver;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public Joystick js1;
  private Drivetrain m_drivetrain;
  private ArcadeDrive m_arcadeDrive;
  private SendableChooser<Command> autoMode;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    js1 = new Joystick(Constants.Controller.CONTROLLER_0);
    m_drivetrain = new Drivetrain();;
    m_arcadeDrive = new ArcadeDrive(m_drivetrain, js1);
    autoMode = new SendableChooser<Command>();

    // Configure the button bindings
    configureButtonBindings();

    autoMode.setDefaultOption("default", new InstantCommand());
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto10.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto11.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto12.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto20.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto21.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto22.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto30.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto31.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto32.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto40.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto41.json"));
    autoMode.addOption("auto10", new Pathweaver(m_drivetrain, "auto42.json"));
  }

  public void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_arcadeDrive);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Button aButton = new JoystickButton(js1, Constants.Controller.CONTROLLER_A);
    Button bButton = new JoystickButton(js1, Constants.Controller.CONTROLLER_B);
    //Button xButton = new JoystickButton(js1, Constants.Controller.X_BUTTON);
    //Button yButton = new JoystickButton(js1, Constants.Controller.Y_BUTTON);
    
    bButton.whenPressed(m_drivetrain::resetEncoders);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(() -> autoMode.getSelected());
    }
}
