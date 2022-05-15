package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7; // import the number 7
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private WPI_TalonFX frontRight;
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backRight;
  private WPI_TalonFX backLeft;
  private AHRS gyro;

  private Field2d m_field2d;
  private DifferentialDriveOdometry m_odometry;

  private TalonFXSimCollection leftSim;
  private TalonFXSimCollection rightSim;

  private DifferentialDrivetrainSim driveSim;
  private SimDouble simAngle;
  private boolean m_isReal;

  /**
   * Creates a Drivetrain subsystem with 4 Falcon 500 TalonFX motors arranged 2 on
   * a side.
   * 
   * @param isReal : True is this code is running on a RoboRio, false if running
   *               in simulation.
   */
  public Drivetrain(boolean isReal) {
    m_isReal = isReal;

    m_field2d = new Field2d();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d(0, 0, new Rotation2d(0)));

    frontRight = new WPI_TalonFX(Constants.Ports.FRONT_RIGHT_PORT);
    frontLeft = new WPI_TalonFX(Constants.Ports.FRONT_LEFT_PORT);
    backRight = new WPI_TalonFX(Constants.Ports.BACK_RIGHT_PORT);
    backLeft = new WPI_TalonFX(Constants.Ports.BACK_LEFT_PORT);

    frontRight.configFactoryDefault();
    frontLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    backLeft.configFactoryDefault();

    frontRight.configVoltageCompSaturation(Constants.MAX_VOLTAGE_DRAW_VOLTS);
    frontLeft.configVoltageCompSaturation(Constants.MAX_VOLTAGE_DRAW_VOLTS);
    backRight.configVoltageCompSaturation(Constants.MAX_VOLTAGE_DRAW_VOLTS);
    backLeft.configVoltageCompSaturation(Constants.MAX_VOLTAGE_DRAW_VOLTS);

    frontRight.enableVoltageCompensation(true);
    frontLeft.enableVoltageCompensation(true);
    backRight.enableVoltageCompensation(true);
    backLeft.enableVoltageCompensation(true);

    gyro = new AHRS();

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(false);
    backRight.setInverted(false);

    if (!isReal) {
      driveSim = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2),
          Constants.Drive.DRIVE_GEARBOX_RATIO,
          Constants.MOMENT_KG_PER_M2,
          Constants.MASS_KG,
          Constants.Drive.DRIVE_WHEEL_DIA_METERS,
          Constants.Drive.DRIVE_TRACK_WIDTH_METERS,
          null);

      rightSim = frontRight.getSimCollection();
      leftSim = frontLeft.getSimCollection();
      simAngle = new SimDouble(
          SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

      resetEncoders();
      resetGyroAngle();
    }
  }

  /**
   * Returns a sendable Field2d object for display on dashboards.
   * 
   * @return The Field2d object, with robot pose from odometry.
   */
  public Field2d getField() {
    return m_field2d;
  }

  /**
   * Sets the position of the two measured motor encoders to zero.
   */
  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  /**
   * Returns the robot's field relative pose in X (meters), Y (meters), and theta
   * (radians, counterclockwise from positive X axis).
   * 
   * @return The Pose2d of the robot from odometry.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Sets the field-relative pose of the robot.
   * 
   * @param xPos  : Field-relative X position (meters).
   * @param yPos  : Field-relative Y position (meters).
   * @param angle : Field-relative angle (radians, counterclockwise from positive
   *              X axis).
   */
  public void resetPose(double xPos, double yPos, Rotation2d angle) {
    resetEncoders();

    m_odometry.resetPosition(
        new Pose2d(new Translation2d(xPos, yPos), angle),
        new Rotation2d(Units.degreesToRadians(90 - getGyroAngle())));

    // sets the velocity of the drive simulation to zero too
    if (!m_isReal) {
      Matrix<N7, N1> stateMatrix = new Matrix<>(Nat.N7(), Nat.N1());
      stateMatrix.set(3, 0, 0);
      stateMatrix.set(4, 0, 0);
      driveSim.setState(stateMatrix);
    }
  }

  /**
   * Sets the left and right values for the drivetrain motors.
   * 
   * @param right : Value between [-1,1] (inclusive) to set the right-side motors
   *              to.
   * @param left  : Value between [-1,1] (inclusive) to set the left-side motors
   *              to.
   */
  public void basicMove(double right, double left) {
    frontLeft.set(left);
    frontRight.set(right);

    if (!m_isReal) {
      driveSim.setInputs(left * Constants.MAX_VOLTAGE_DRAW_VOLTS, right * Constants.MAX_VOLTAGE_DRAW_VOLTS);
    }
  }

  /**
   * Sets the motors of the drivetrain to a specified neutral mode to go to when
   * an input of zero or null is recieved.
   * <p>
   * NeutralMode.Brake : The motors will attempt to go to zero velocity.
   * <p>
   * NeutralMode.Coast : The motors will not apply current.
   * 
   * @param mode : The mode to set to.
   */
  public void setMotorNeutralMode(NeutralMode mode) {
    frontLeft.setNeutralMode(mode);
    frontRight.setNeutralMode(mode);
    backLeft.setNeutralMode(mode);
    backRight.setNeutralMode(mode);
  }

  /**
   * Returns the angle of the onboard gyroscope.
   * 
   * @return The angle output by the gyroscope (degrees, clockwise from positive
   *         Y axis).
   */
  public double getGyroAngle() {
    if (m_isReal) {
      return gyro.getAngle();
    } else {
      return simAngle.get();
    }
  }

  /**
   * Sets the angle of the gyroscope to zero. To maintain a proper field-relative
   * angle, do not call this during a match.
   */
  public void resetGyroAngle() {
    if (m_isReal) {
      gyro.reset();
    } else {
      simAngle.set(0);
    }
  }

  /**
   * Returns the displacement of the right encoder.
   * 
   * @return The position of the measuring right motor encoder (encoder ticks).
   */
  public double getRightSensorPosition() {
    return frontRight.getSelectedSensorPosition();
  }

  /**
   * Returns the displacement of the left encoder.
   * 
   * @return The position of the measuring left motor encoder (encoder ticks).
   */
  public double getLeftSensorPosition() {
    return -frontLeft.getSelectedSensorPosition();
  }

  /**
   * Returns the rate of the right encoder.
   * 
   * @return The velocity of the measuring right motor encoder (encoder ticks
   *         per second).
   */
  public double getRightSensorVelocity() {
    return 10 * frontRight.getSelectedSensorVelocity();
  }

  /**
   * Returns the rate of the left encoder.
   * 
   * @return The velocity of the measuring left motor encoder (encoder ticks per
   *         second).
   */
  public double getLeftSensorVelocity() {
    return -10 * frontLeft.getSelectedSensorVelocity();
  }

  /**
   * Updates the approximate pose of the robot with new encoder and gyro
   * measurements. This should be called as often as possible.
   */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(Units.degreesToRadians(90 - getGyroAngle())),
        getLeftSensorPosition() * Constants.Drive.DRIVE_METERS_PER_COUNT,
        getRightSensorPosition() * Constants.Drive.DRIVE_METERS_PER_COUNT);

    m_field2d.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putData("Field", m_field2d);
    SmartDashboard.putNumber("left actual v",
        getLeftSensorVelocity() * Constants.Drive.DRIVE_METERS_PER_COUNT);
    SmartDashboard.putNumber("right actual v",
        getRightSensorVelocity() * Constants.Drive.DRIVE_METERS_PER_COUNT);
  }

  @Override
  public void simulationPeriodic() {
    driveSim.update(Constants.LOOPTIME_SECS);

    leftSim.setIntegratedSensorRawPosition(
        (int) (driveSim.getLeftPositionMeters() / Constants.Drive.DRIVE_METERS_PER_COUNT));
    rightSim.setIntegratedSensorRawPosition(
        (int) (driveSim.getRightPositionMeters() / Constants.Drive.DRIVE_METERS_PER_COUNT));
    leftSim.setIntegratedSensorVelocity(
        (int) (driveSim.getLeftVelocityMetersPerSecond() / Constants.Drive.DRIVE_METERS_PER_COUNT));
    rightSim.setIntegratedSensorVelocity(
        (int) (driveSim.getLeftVelocityMetersPerSecond() / Constants.Drive.DRIVE_METERS_PER_COUNT));
    simAngle.set(driveSim.getHeading().getDegrees());
  }
}