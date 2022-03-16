package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public static WPI_TalonFX frontRight;
  public static WPI_TalonFX frontLeft;
  private static WPI_TalonFX backRight;
  private static WPI_TalonFX backLeft;
  public static AHRS gyro;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    frontRight = new WPI_TalonFX(Constants.Ports.FRONT_RIGHT); // constructs motor controllers and
    frontLeft = new WPI_TalonFX(Constants.Ports.FRONT_LEFT); // assigns them to the correct
    backRight = new WPI_TalonFX(Constants.Ports.BACK_RIGHT); // device ID
    backLeft = new WPI_TalonFX(Constants.Ports.BACK_LEFT);

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
  }
  
  public void resetEncoders(){
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public void tankDriveMove(double speed, double rotation) {
    double left_throttle = (speed - rotation); // equations for a differential drive
    double right_throttle = (speed + rotation); // pass in raw controller values

    frontLeft.set(left_throttle);
    frontRight.set(right_throttle);
  }

  public void basicMove(double right, double left) { // handy little wrapper for diffDrive
    frontLeft.set(left);               // pass in processed values
    frontRight.set(right);
  }
  
  public static void setMotorNeutralMode(NeutralMode mode) {
    frontLeft.setNeutralMode(mode);
    frontRight.setNeutralMode(mode);
    backLeft.setNeutralMode(mode);
    backRight.setNeutralMode(mode);
  }

 @Override
 public void periodic() {
  // This method will be called once per scheduler run
  SmartDashboard.putNumber("Delta Drive L Encoder", frontLeft.getSelectedSensorVelocity() * Constants.DRIVE_IN_PER_COUNT * 0.1);
  SmartDashboard.putNumber("Delta Drive R Encoder", frontRight.getSelectedSensorVelocity() * Constants.DRIVE_IN_PER_COUNT * 0.1);
  SmartDashboard.putNumber("Drive L Encoder", frontLeft.getSelectedSensorPosition() * Constants.DRIVE_IN_PER_COUNT);
  SmartDashboard.putNumber("Drive R Encoder", frontRight.getSelectedSensorPosition() * Constants.DRIVE_IN_PER_COUNT);
 }
}