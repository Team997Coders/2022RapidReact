// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.auto.AutoBackGrabBall;
import frc.robot.commands.auto.AutoDistance;
import frc.robot.commands.auto.AutoDriveToDistanceIntake;
import frc.robot.commands.auto.AutoRotate;
import frc.robot.commands.auto.AutoBallDump;
import frc.robot.commands.climb.SimpleClimb;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.lighting.Spartan1;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
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

  private Joystick jsDrive;
  private JoystickButton intakeForwardButton;
  private JoystickButton intakeBackwardsButton;

  private SimpleClimb m_simpleClimb;
  private ArcadeDrive m_arcadeDrive;
  private IntakeCommand m_intakeCommand;
  private Spartan1 m_defaultLighting;

  private PowerDistribution m_pdp;
  private Climber m_climber;
  private Drivetrain m_drive;
  private Intake m_intake;
  private Lighting m_lighting;
  private SendableChooser<Command> autoModeSwitcher;
  private SendableChooser<Command> ledModeSwitcher;

  private IntakeCommand intakeForwardsCommand;
  private IntakeCommand intakeBackwardsCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pdp = new PowerDistribution();

    jsDrive = new Joystick(Constants.Controller.CONTROLLER_0);

    m_climber = new Climber();
    m_drive = new Drivetrain();
    m_intake = new Intake();
    m_lighting = new Lighting(Constants.Lighting.LED_COUNT);

    m_simpleClimb = new SimpleClimb(m_climber, 
      () -> { return jsDrive.getRawAxis(Constants.Controller.TRIGGER_CLIMB_UP); }, 
      () -> { return jsDrive.getRawAxis(Constants.Controller.TRIGGER_CLIMB_DN); },
      () -> { return jsDrive.getRawButton(Constants.Controller.A_BUTTON); });
    m_arcadeDrive = new ArcadeDrive(m_drive,
      () -> { return -jsDrive.getRawAxis(Constants.Controller.JOYSTICK_1); },
      () -> { return -jsDrive.getRawAxis(Constants.Controller.JOYSTICK_2); },
      () -> { return jsDrive.getRawButton(Constants.Controller.RIGHT_BUMPER); });

    m_defaultLighting = new Spartan1(m_lighting, Constants.Lighting.DEFAULT_ALTERNATING_TIME_MS);

    autoModeSwitcher = new SendableChooser<Command>();
    ledModeSwitcher = new SendableChooser<Command>();
    
    m_pdp.clearStickyFaults();

    // Configure the button bindings
    configureButtonBindings();

    autoModeSwitcher.setDefaultOption("None", new InstantCommand());
    autoModeSwitcher.addOption("Ball Dump: Leave Tarmac", new AutoBallDump(m_drive, m_intake, -96, 4000));
    autoModeSwitcher.addOption("Ball Dump: Stay In Position", new AutoBallDump(m_drive, m_intake, 0, 5000));
    autoModeSwitcher.addOption("Leave Tarmac: Side Position", new AutoDistance(m_drive, 60, 5000));
    autoModeSwitcher.addOption("Leave Tarmac: Center Position", new AutoDistance(m_drive, 90, 5000));
    autoModeSwitcher.addOption("Collect Auto Full", new AutoBackGrabBall(m_drive, m_intake, m_pdp, 120));
    autoModeSwitcher.addOption("Test turn", new AutoRotate(m_drive, 135));
    autoModeSwitcher.addOption("Drive Intake Test", new AutoDriveToDistanceIntake(m_drive, m_intake, m_pdp, 36, 3000));
    Shuffleboard.getTab("Autonomous").add(autoModeSwitcher);

    //ledModeSwitcher.setDefaultOption("Default Spartan", new Spartan1(m_lighting, Constants.Lighting.DEFAULT_ALTERNATING_TIME_MS));
    //ledModeSwitcher.addOption("Alliance Colors", new AllianceColors(m_lighting));
    //Shuffleboard.getTab("LEDs").add(ledModeSwitcher);

    CameraServer.startAutomaticCapture().setResolution(100, 100);
  }

  public void setLeds() {
    CommandScheduler.getInstance().setDefaultCommand(m_lighting, m_defaultLighting);
  }

  public void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_arcadeDrive); //sets the Drivetrain to default to ArcadeDrive in teleop
    CommandScheduler.getInstance().setDefaultCommand(m_climber, m_simpleClimb); //sets the Climber to default to SimpleClimb in teleop
    CommandScheduler.getInstance().setDefaultCommand(m_intake, m_intakeCommand);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeForwardButton = new JoystickButton(jsDrive, Constants.Controller.A_BUTTON);
    intakeBackwardsButton = new JoystickButton(jsDrive, Constants.Controller.B_BUTTON);

    intakeForwardButton.whileHeld(intakeForwardsCommand = new IntakeCommand(m_intake, Constants.Intake.INTAKE_DEFAULT_SPEED));
    intakeBackwardsButton.cancelWhenPressed(intakeForwardsCommand);

    intakeBackwardsButton.whileHeld(intakeBackwardsCommand = new IntakeCommand(m_intake, -Constants.Intake.INTAKE_DEFAULT_SPEED));
    intakeForwardButton.cancelWhenPressed(intakeBackwardsCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoModeSwitcher.getSelected();
  }

  public Command getLedCommand() {
    return ledModeSwitcher.getSelected();
  }

  public void SetDriveNeutralMode(NeutralMode mode) {
    m_drive.setMotorNeutralMode(mode);
  }
}
