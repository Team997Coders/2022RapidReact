// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.SimpleClimb;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// TODO: test coast/brake mode stuff
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static Joystick js1;
  //public JoystickButton neutralModeToggleButton;
  public JoystickButton resetClimbEncoderButton;
  public JoystickButton resetDriveEncodersButton;
  private Climber m_climber;
  private SimpleClimb m_simpleClimb;
  private ArcadeDrive m_arcadeDrive;
  private Drivetrain m_drive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    js1 = new Joystick(Constants.Controller.CONTROLLER_0);
    m_climber = new Climber();
    m_drive = new Drivetrain();
    m_simpleClimb = new SimpleClimb(m_climber, js1);
    m_arcadeDrive = new ArcadeDrive(m_drive);
    // Configure the button bindings
    configureButtonBindings();
    
  }

  public void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_arcadeDrive); //sets the Drivetrain to default to ArcadeDrive in teleop
    CommandScheduler.getInstance().setDefaultCommand(m_climber, m_simpleClimb); //sets the Climber to default to SimpleClimb in teleop
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    resetClimbEncoderButton = new JoystickButton(js1, Constants.Controller.X_BUTTON); // when X is pressed, distance on the climber NEO encoder is zeroed, for testing
    resetDriveEncodersButton = new JoystickButton(js1, Constants.Controller.B_BUTTON); // when B is pressed, distance on drive encoders is zeroed
    // neutralModeToggleButton = new JoystickButton(js1, Constants.Controller.Y_BUTTON); // when Y is pressed, the drivetrain goes to coast mode

    resetDriveEncodersButton.whenPressed(m_drive::resetEncoders);
    resetClimbEncoderButton.whenPressed(m_climber::resetEncoder);
    //neutralModeToggleButton.whenPressed(m_drive::setMotorModeCoast);
    //neutralModeToggleButton.whenReleased(m_drive::setMotorModeBrake);
  }

  public static double joystickLeftInput() {
    if (Math.abs(js1.getRawAxis(Constants.Controller.JOYSTICK_1)) < Constants.Controller.DEAD_ZONE_SENSITIVITY) {
      return 0;                        // these methods are here so that the deadzone code doesn't
    } else {                           // have to be repeatedly copied-- static to avoid constructing a
      return js1.getRawAxis(Constants.Controller.JOYSTICK_1);  // RobotContainer in each subsystem
    }
  }
  public static double joystickRightInput() {
    if (Math.abs(js1.getRawAxis(Constants.Controller.JOYSTICK_2)) < Constants.Controller.DEAD_ZONE_SENSITIVITY) {
      return 0;
    } else {
      return js1.getRawAxis(Constants.Controller.JOYSTICK_2);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  public void disabledInit() {
    //m_drive.setMotorModeBrake();
  }

  public void autonomousInit() {
    //m_drive.setMotorModeCoast();
  }

  public void teleopInit() {
    //m_drive.setMotorModeBrake();
  }
}
