package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public WPI_TalonFX frontRight;
  public WPI_TalonFX frontLeft;
  private WPI_TalonFX backRight;
  private WPI_TalonFX backLeft;
  public static AHRS gyro;

  
  private MotorControllerGroup leftSide;
  private MotorControllerGroup rightSide;
  private DifferentialDrive diffDrive;
  /** Creates a new Drivetrain. */
  public Drivetrain() {

    frontRight = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[2]); // constructs motor controllers and
    frontLeft = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[0]); // assigns them to the correct
    backRight = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[3]); // device ID
    backLeft = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[1]);

    gyro = new AHRS(); // constructs the IMU

    frontRight.configFactoryDefault();
    frontLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    backLeft.configFactoryDefault();

    backRight.follow(frontRight); // we only have to handle two motors, not four
    backLeft.follow(frontLeft); // in a tank drive, they do the same thing

    frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30); //sets up encoders
    frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    resetEncoders(); // makes sure they're at zero

    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    leftSide = new MotorControllerGroup(frontLeft, backLeft);
    rightSide = new MotorControllerGroup(frontRight, backRight);

    diffDrive = new DifferentialDrive(leftSide, rightSide);
    
  }
  
  public void resetEncoders(){
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public void tankDriveMove(double speed, double rotation) {
    double left_throttle = (speed-rotation); // equations for a differential drive
    double right_throttle = (speed+rotation); // pass in raw controller values

    diffDrive.tankDrive(left_throttle, right_throttle); // moves motors
  }

  public void basicMove(double right, double left) { // handy little wrapper for diffDrive
    diffDrive.tankDrive(left, right);                // pass in processed values
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Delta Drive L Encoder", frontLeft.getSelectedSensorVelocity()*Constants.DRIVE_IN_PER_COUNT);
    SmartDashboard.putNumber("Delta Drive R Encoder", frontRight.getSelectedSensorVelocity()*Constants.DRIVE_IN_PER_COUNT);
    SmartDashboard.putNumber("Drive L Encoder", frontLeft.getSelectedSensorPosition()*Constants.DRIVE_IN_PER_COUNT);
    SmartDashboard.putNumber("Drive R Encoder", frontRight.getSelectedSensorPosition()*Constants.DRIVE_IN_PER_COUNT);
  }
  public void setMotorModeCoast() { // for auto-- PIDs don't like brake mode
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }
  public void setMotorModeBrake() { // for teleop safety/ease of drive
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
  }
}
