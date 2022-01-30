package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.utilties.ResetEncoders;

public class Drivetrain extends SubsystemBase {
  public static WPI_TalonFX frontRight;
  public static WPI_TalonFX frontLeft;
  private static WPI_TalonFX backRight;
  private static WPI_TalonFX backLeft;
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

    ResetEncoders.resetEncoders();

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

  public void newArcadeDrive(double speed, double rotation) {
    double left_throttle = (speed-rotation);
    double right_throttle = (speed+rotation);
    SmartDashboard.putNumber("lThrottle", left_throttle);
    SmartDashboard.putNumber("rThrottle", right_throttle);


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
  public static void setMotorModeCoast() {
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }
  public static void setMotorModeBrake() {
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
  }
}
