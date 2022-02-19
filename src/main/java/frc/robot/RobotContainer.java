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
import frc.robot.commands.AutoDistance;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.ClimberPID;
import frc.robot.commands.Pathweaver;
import frc.robot.commands.SimpleClimb;
import frc.robot.subsystems.Climber;
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
  private Climber m_climber;
  private Drivetrain m_drivetrain;
  private ClimberPID m_climberPID;
  private SimpleClimb m_simpleClimb;
  private ArcadeDrive m_arcadeDrive;
  private AutoDistance m_autoDistance;
  private AutoRotate m_autoRotate;
  private SendableChooser<Command> autoMode;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    js1 = new Joystick(Constants.Controller.CONTROLLER_1);
    m_climber = new Climber();
    m_drivetrain = new Drivetrain();
    m_climberPID = new ClimberPID(m_climber, js1);
    m_simpleClimb = new SimpleClimb(m_climber, js1);
    m_arcadeDrive = new ArcadeDrive(m_drivetrain, js1);
    autoMode = new SendableChooser<Command>();

    // Configure the button bindings
    configureButtonBindings();


  }

  public void setDefaultCommands() {
    //CommandScheduler.getInstance().setDefaultCommand(m_drive, m_arcadedrive);
    //CommandScheduler.getInstance().setDefaultCommand(m_climber, m_simpleClimb);
    CommandScheduler.getInstance().setDefaultCommand(m_climber, m_climberPID);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Button aButton = new JoystickButton(js1, Constants.Controller.A_BUTTON);
    Button bButton = new JoystickButton(js1, Constants.Controller.B_BUTTON);
    //Button xButton = new JoystickButton(js1, Constants.Controller.X_BUTTON);
    //Button yButton = new JoystickButton(js1, Constants.Controller.Y_BUTTON);
    
    bButton.whenPressed(m_drivetrain::resetEncoders);
    aButton.whenPressed(m_climberPID::reDisplayClimberPidGains);
  }
  private final Command auto10 = new Pathweaver(m_drivetrain, "auto10.json");
  private final Command auto11 = new Pathweaver(m_drivetrain, "auto11.json");
  private final Command auto12 = new Pathweaver(m_drivetrain, "auto12.json");
  private final Command auto20 = new Pathweaver(m_drivetrain, "auto20.json");
  private final Command auto21 = new Pathweaver(m_drivetrain, "auto21.json");
  private final Command auto22 = new Pathweaver(m_drivetrain, "auto22.json");
  private final Command auto30 = new Pathweaver(m_drivetrain, "auto30.json");
  private final Command auto31 = new Pathweaver(m_drivetrain, "auto31.json");
  private final Command auto32 = new Pathweaver(m_drivetrain, "auto32.json");
  private final Command auto40 = new Pathweaver(m_drivetrain, "auto40.json");
  private final Command auto41 = new Pathweaver(m_drivetrain, "auto41.json");
  private final Command auto42 = new Pathweaver(m_drivetrain, "auto42.json");
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(() -> m_climber.climberMove(0.4)); //test
    }
}
