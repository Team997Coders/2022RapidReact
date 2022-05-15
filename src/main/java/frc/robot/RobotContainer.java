// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.auto.AutoDistance;
import frc.robot.commands.auto.AutoRotate;
import frc.robot.commands.auto.RamseteFollower;
import frc.robot.commands.auto.TimedDrive;
import frc.robot.commands.climb.SimpleClimb;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.intake.SimpleIntake;
import frc.robot.commands.climb.CheckpointClimb;
import frc.robot.commands.lighting.AllianceColorsFlashing;
import frc.robot.commands.lighting.AllianceColorsStatic;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.Constants.Controller.JoystickMappingTypes;
import frc.robot.Constants.Drive.DriveCommandModes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Joystick jsDrive;

  private SimpleClimb m_simpleClimb;
  private TeleopDriveCommand m_teleopDrive;
  private SimpleIntake m_simpleIntake;
  private CheckpointClimb m_checkpointClimber;

  private JoystickButton startCheckpointClimbButton;
  private JoystickButton endCheckpointClimbButton;

  private Climber m_climber;
  private Drivetrain m_drive;
  private Intake m_intake;
  private Lighting m_lighting;

  private SendableChooser<Command> autoModeSwitcher;
  private SendableChooser<Command> ledModeSwitcher;
  private SendableChooser<Boolean> demoModeSwitcher;
  private SendableChooser<DriveCommandModes> driveModeSwitcher;
  private SendableChooser<JoystickMappingTypes> joystickMappingSwitcher;

  private PowerDistribution m_pdp;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // this constructor is unreadable unless split up like so
    constructSubsystems();

    constructCommands();

    setupSelectors();

    if (Robot.isReal()) {
      CameraServer.startAutomaticCapture().setResolution(100, 100);
    }
  }

  /**
   * Constructs the hardware of the robot.
   */
  private void constructSubsystems() {
    m_pdp = new PowerDistribution();

    jsDrive = new Joystick(Constants.Controller.CONTROLLER_0);

    m_climber = new Climber();
    m_drive = new Drivetrain(Robot.isReal());
    m_intake = new Intake();
    m_lighting = new Lighting(Constants.Lighting.LED_COUNT);
  }

  /**
   * Constructs the commands of the robot.
   */
  private void constructCommands() {
    m_simpleClimb = new SimpleClimb(m_climber,
        () -> {
          return jsDrive.getRawAxis(Constants.Controller.TRIGGER_CLIMB_UP);
        },
        () -> {
          return jsDrive.getRawAxis(Constants.Controller.TRIGGER_CLIMB_DN);
        },
        () -> {
          return jsDrive.getRawButton(Constants.Controller.A_BUTTON);
        });

    m_teleopDrive = new TeleopDriveCommand(m_drive,
        () -> {
          return -jsDrive.getRawAxis(Constants.Controller.JOYSTICK_1);
        },
        () -> {
          return -jsDrive.getRawAxis(Constants.Controller.JOYSTICK_2);
        },
        () -> {
          return demoModeSwitcher.getSelected();
        },
        Robot.isReal(),
        () -> {
          return driveModeSwitcher.getSelected();
        }, () -> {
          return joystickMappingSwitcher.getSelected();
        });

    m_simpleIntake = new SimpleIntake(m_intake,
        () -> {
          return jsDrive.getRawButton(Constants.Controller.LEFT_BUMPER);
        },
        () -> {
          return jsDrive.getRawButton(Constants.Controller.RIGHT_BUMPER);
        });

    m_checkpointClimber = new CheckpointClimb(m_climber,
        () -> {
          return jsDrive.getRawButtonPressed(Constants.Controller.X_BUTTON);
        },
        () -> {
          return jsDrive.getRawButtonPressed(Constants.Controller.B_BUTTON);
        });
  }

  /**
   * Gives options to the various selectors and displays them to Shuffleboard.
   */
  private void setupSelectors() {
    // temporary location for this test trajectory
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        List.of(new Translation2d(1.5, 0),
            new Translation2d(3, 1.5)),
        new Pose2d(new Translation2d(4.5, 3), new Rotation2d(0)),
        new TrajectoryConfig(1, 0.5));

    autoModeSwitcher = new SendableChooser<Command>();
    ledModeSwitcher = new SendableChooser<Command>();
    demoModeSwitcher = new SendableChooser<Boolean>();
    driveModeSwitcher = new SendableChooser<DriveCommandModes>();
    joystickMappingSwitcher = new SendableChooser<JoystickMappingTypes>();

    m_pdp.clearStickyFaults();

    // Configure the button bindings
    configureButtonBindings();

    autoModeSwitcher.setDefaultOption("None", new InstantCommand());
    autoModeSwitcher.addOption("Test Turn", new AutoRotate(m_drive, 90, 3000));
    autoModeSwitcher.addOption("Test Drive", new AutoDistance(m_drive, 36, 3000));
    autoModeSwitcher.addOption("Ramsete Test Trajectory",
        new RamseteFollower(testTrajectory, m_drive, 0, 0, new Rotation2d(0), testTrajectory.getTotalTimeSeconds())
            .andThen(new TimedDrive(m_drive, 0, 0, 5000)));

    ledModeSwitcher.setDefaultOption("Static", new AllianceColorsStatic(m_lighting));
    ledModeSwitcher.addOption("Flashing",
        new AllianceColorsFlashing(m_lighting, Constants.Lighting.DEFAULT_ALTERNATING_TIME_S));

    if (Constants.DEMO_MODE_DEFAULT) {
      demoModeSwitcher.setDefaultOption("Demo Mode Enabled", true);
      demoModeSwitcher.addOption("Demo Mode Disabled", false);
    } else {
      demoModeSwitcher.setDefaultOption("Demo Mode Disabled", false);
      demoModeSwitcher.addOption("Demo Mode Enabled", true);
    }

    if (Constants.Drive.DEFAULT_DRIVE_MODE == DriveCommandModes.ARCADE_DRIVE) {
      driveModeSwitcher.setDefaultOption("Arcade Drive", DriveCommandModes.ARCADE_DRIVE);
      driveModeSwitcher.addOption("Curvature Drive", DriveCommandModes.CURVATURE_DRIVE);
      driveModeSwitcher.addOption("Mixed Arcade-Curvature", DriveCommandModes.MIXED_ARCADE_CURVATURE);
    } else if (Constants.Drive.DEFAULT_DRIVE_MODE == DriveCommandModes.CURVATURE_DRIVE) {
      driveModeSwitcher.setDefaultOption("Curvature Drive", DriveCommandModes.CURVATURE_DRIVE);
      driveModeSwitcher.addOption("Arcade Drive", DriveCommandModes.ARCADE_DRIVE);
      driveModeSwitcher.addOption("Mixed Arcade-Curvature", DriveCommandModes.MIXED_ARCADE_CURVATURE);
    } else if (Constants.Drive.DEFAULT_DRIVE_MODE == DriveCommandModes.MIXED_ARCADE_CURVATURE) {
      driveModeSwitcher.setDefaultOption("Mixed Arcade-Curvature", DriveCommandModes.MIXED_ARCADE_CURVATURE);
      driveModeSwitcher.addOption("Arcade Drive", DriveCommandModes.ARCADE_DRIVE);
      driveModeSwitcher.addOption("Curvature Drive", DriveCommandModes.CURVATURE_DRIVE);
    }

    if (Constants.Controller.DEFAULT_MAPPING_TYPE == JoystickMappingTypes.LINEAR) {
      joystickMappingSwitcher.setDefaultOption("Linear Mapping", JoystickMappingTypes.LINEAR);
      joystickMappingSwitcher.addOption("Quadratic Mapping", JoystickMappingTypes.QUADRATIC);
    } else if (Constants.Controller.DEFAULT_MAPPING_TYPE == JoystickMappingTypes.QUADRATIC) {
      joystickMappingSwitcher.setDefaultOption("Quadratic Mapping", JoystickMappingTypes.QUADRATIC);
      joystickMappingSwitcher.addOption("Linear Mapping", JoystickMappingTypes.LINEAR);
    }

    Shuffleboard.getTab("Selectors").add(autoModeSwitcher);
    Shuffleboard.getTab("Selectors").add(ledModeSwitcher);
    Shuffleboard.getTab("Selectors").add(demoModeSwitcher);
    Shuffleboard.getTab("Selectors").add(driveModeSwitcher);
    Shuffleboard.getTab("Selectors").add(joystickMappingSwitcher);
  }

  /**
   * Sets the default commands for each subsystem.
   */
  public void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_teleopDrive);
    CommandScheduler.getInstance().setDefaultCommand(m_climber, m_simpleClimb);
    CommandScheduler.getInstance().setDefaultCommand(m_intake, m_simpleIntake);
    CommandScheduler.getInstance().setDefaultCommand(m_lighting, ledModeSwitcher.getSelected());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    startCheckpointClimbButton = new JoystickButton(jsDrive, Constants.Controller.A_BUTTON);
    endCheckpointClimbButton = new JoystickButton(jsDrive, Constants.Controller.Y_BUTTON);

    startCheckpointClimbButton.cancelWhenActive(m_simpleClimb);
    startCheckpointClimbButton.whenPressed(m_checkpointClimber);

    endCheckpointClimbButton.cancelWhenActive(m_checkpointClimber);
    endCheckpointClimbButton.whenPressed(m_simpleClimb);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoModeSwitcher.getSelected();
  }

  /**
   * {@link Drivetrain#setMotorNeutralMode(mode)}, but in RobotContainer.
   * 
   * @param mode The NeutralMode.mode to set to.
   */
  public void SetDriveNeutralMode(NeutralMode mode) {
    m_drive.setMotorNeutralMode(mode);
  }

  /**
   * Whatever a sticky fault is, this clears them.
   */
  public void clearPDPStickyFaults() {
    m_pdp.clearStickyFaults();
  }

  /**
   * Gets voltage of the robot's electrical systems at the PDP.
   * 
   * @return The voltage in volts.
   */
  public double getPDPVoltage() {
    return m_pdp.getVoltage();
  }
}
