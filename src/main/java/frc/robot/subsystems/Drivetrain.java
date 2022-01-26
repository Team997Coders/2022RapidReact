package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;

public class Drivetrain extends SubsystemBase {
  private static WPI_TalonFX frontRight;
  private static WPI_TalonFX frontLeft;
  private WPI_TalonFX backRight;
  private WPI_TalonFX backLeft;
  private AHRS gyro;

  
  private MotorControllerGroup leftSide;
  private MotorControllerGroup rightSide;
  private DifferentialDrive diffDrive;
  /** Creates a new Drivetrain. */
  public Drivetrain() {

    frontRight = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[2]);
    frontLeft = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[0]);
    backRight = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[3]);
    backLeft = new WPI_TalonFX(Constants.Ports.DRIVE_PORTS[1]);

    gyro = new AHRS();

    frontRight.configFactoryDefault();
    frontLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    backLeft.configFactoryDefault();

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    ResetEncoders();

    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    leftSide = new MotorControllerGroup(frontLeft, backLeft);
    rightSide = new MotorControllerGroup(frontRight, backRight);

    diffDrive = new DifferentialDrive(leftSide, rightSide);
    
  }

  public static void ResetEncoders(){
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  // public void arcadeDrive(double speed, double rotation){
  //  diffDrive.arcadeDrive(speed, rotation);
  // }

  public double numberLimits(double f, boolean ceiling, double highestAbs, boolean deadMan, double deadManTolerance) {  // aww yeah simple function with 5 arguments
    if (deadMan == true) {
      if (Math.abs(f) < deadManTolerance) {
        f = 0;
      }
    }
    if (ceiling == true) {
      if (f > highestAbs) {
        f = highestAbs;
      }
      else if (f < -1*highestAbs) {
        f = -1*highestAbs;
      }
    }
    return f;
  }
  public void betterArcadeDrive(double speed, double rotation) {
    speed = numberLimits(speed, true, 1, true, 0.1);
    rotation = numberLimits(rotation, true, 1, true, 0.1);

    double left_throttle = (numberLimits(speed, true, 1, false, 0))-numberLimits(rotation, true, 1, false, 0);
    double right_throttle = (numberLimits(speed, true, 1, false, 0))+numberLimits(rotation, true, 1, false, 0);

    SmartDashboard.putNumber("lThrottle", left_throttle);
    SmartDashboard.putNumber("rThrottle", right_throttle);
    SmartDashboard.putNumber("lJoystick", ArcadeDrive.js1.getRawAxis(Constants.Ports.JOYSTICK_1));
    SmartDashboard.putNumber("rJoystick", ArcadeDrive.js1.getRawAxis(Constants.Ports.JOYSTICK_2));

    //leftSide.set(left_throttle);
    //rightSide.set(right_throttle);
    diffDrive.tankDrive(left_throttle, right_throttle);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("L Encoder V", frontLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("R Encoder V", frontRight.getSelectedSensorVelocity());
    SmartDashboard.putNumber("NavX Heading", gyro.getAngle());

    SmartDashboard.putNumber("L Encoder Distance", frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("R Encoder Distance", frontRight.getSelectedSensorPosition());
  }
}