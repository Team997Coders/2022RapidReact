// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.auto.BallDumpAuto;
import frc.robot.commands.auto.LeaveTarmacAuto;
import frc.robot.commands.climb.SimpleClimb;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
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

  public static Joystick jsDrive;
  public static Joystick jsClimb;
  public JoystickButton resetClimbEncoderButton;
  public JoystickButton resetDriveEncodersButton;
  public static JoystickButton turboModeButton;
  private Climber m_climber;
  private SimpleClimb m_simpleClimb;
  private ArcadeDrive m_arcadeDrive;
  private Drivetrain m_drive;
  private SendableChooser<Command> autoModeSwitcher;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    jsDrive = new Joystick(Constants.Controller.CONTROLLER_0);
    //jsClimb = new Joystick(Constants.Controller.CONTROLLER_1);

    m_climber = new Climber();
    m_drive = new Drivetrain();

    m_simpleClimb = new SimpleClimb(m_climber, 
      () -> { return jsDrive.getRawAxis(Constants.Controller.TRIGGER_CLIMB_UP); }, 
      () -> { return jsDrive.getRawAxis(Constants.Controller.TRIGGER_CLIMB_DN); });
    m_arcadeDrive = new ArcadeDrive(m_drive,
      () -> { return jsDrive.getRawAxis(Constants.Controller.JOYSTICK_1); },
      () -> { return jsDrive.getRawAxis(Constants.Controller.JOYSTICK_2); },
      () -> { return jsDrive.getRawButton(Constants.Controller.RIGHT_BUMPER); });

    autoModeSwitcher = new SendableChooser<Command>();
    // Configure the button bindings
    configureButtonBindings();
    autoModeSwitcher.setDefaultOption("None", new InstantCommand());
    autoModeSwitcher.addOption("Ball Dump: Leave Tarmac", new BallDumpAuto(m_drive, 1));
    autoModeSwitcher.addOption("Ball Dump: Stay In Position", new BallDumpAuto(m_drive, 0));
    autoModeSwitcher.addOption("Leave Tarmac: Side Position", new LeaveTarmacAuto(m_drive, 0));
    autoModeSwitcher.addOption("Leave Tarmac: Center Position", new LeaveTarmacAuto(m_drive, 1));
    Shuffleboard.getTab("Autonomous").add(autoModeSwitcher);

    CameraServer.startAutomaticCapture();
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
    resetClimbEncoderButton = new JoystickButton(jsDrive, Constants.Controller.X_BUTTON); // when X is pressed, distance on the climber NEO encoder is zeroed, for testing
    resetDriveEncodersButton = new JoystickButton(jsDrive, Constants.Controller.B_BUTTON); // when B is pressed, distance on drive encoders is zeroed
    // neutralModeToggleButton = new JoystickButton(js1, Constants.Controller.Y_BUTTON); // when Y is pressed, the drivetrain goes to coast mode
    turboModeButton = new JoystickButton(jsDrive, Constants.Controller.A_BUTTON);

    resetDriveEncodersButton.whenPressed(m_drive::resetEncoders);
    resetClimbEncoderButton.whenPressed(m_climber::resetEncoder);
  }

  public static double joystickLeftInput() {
    //if (Math.abs(jsDrive.getRawAxis(Constants.Controller.JOYSTICK_1)) < Constants.Controller.DEAD_ZONE_SENSITIVITY) {
    //  return 0;                        // these methods are here so that the deadzone code doesn't
    //} else {                           // have to be repeatedly copied-- static to avoid constructing a
      return jsDrive.getRawAxis(Constants.Controller.JOYSTICK_1);  // RobotContainer in each subsystem
    //}
  }
  public static double joystickRightInput() {
    //if (Math.abs(jsDrive.getRawAxis(Constants.Controller.JOYSTICK_2)) < Constants.Controller.DEAD_ZONE_SENSITIVITY) {
     // return 0;
    //} else {
      return jsDrive.getRawAxis(Constants.Controller.JOYSTICK_2);
    //}
  }
  public static boolean turboModePressed() {
    return turboModeButton.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoModeSwitcher.getSelected();
  }
}
