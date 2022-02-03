// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Drivetrain;
// public class ArcadeDrive extends CommandBase {
//   /** Creates a new ArcadeDrive. */
//   private Drivetrain drivetrain;
//   double rightSmoothedInput = 0;
//   double leftSmoothedInput = 0;

//   public ArcadeDrive(Drivetrain drive) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(drive);
//     drivetrain = drive;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }
//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     double leftRawInput = RobotContainer.js1.getRawAxis(Constants.Ports.JOYSTICK_1);
//     double rightRawInput = RobotContainer.js1.getRawAxis(Constants.Ports.JOYSTICK_2);

//     double leftSmoothedInputTarget = leftRawInput;
//     double rightSmoothedInputTarget = rightRawInput;
    
//     if (Math.abs(leftSmoothedInputTarget)>1) {
//       if (leftSmoothedInputTarget<0) {
//         leftSmoothedInputTarget = -1;
//       }
//       else if (leftSmoothedInputTarget!=0) {
//         leftSmoothedInputTarget = 1;
//       }
//     }
//     if (Math.abs(rightSmoothedInputTarget)>1) {
//       if (rightSmoothedInputTarget<0) {
//         rightSmoothedInputTarget = -1;
//       }
//       else if (rightSmoothedInputTarget!=0) {
//         rightSmoothedInputTarget = 1;
//       }
//     }
//     if (leftSmoothedInput < leftSmoothedInputTarget) {
//       if (leftSmoothedInputTarget-leftSmoothedInput < Constants.MovementConstants.ACCEL_DAMPING_SLOPE) {
//         leftSmoothedInput = leftSmoothedInputTarget;
//       }
//       else {
//         leftSmoothedInput += Constants.MovementConstants.ACCEL_DAMPING_SLOPE;
//       }
//     }
//     if (leftSmoothedInput > leftSmoothedInputTarget) {
//       if (leftSmoothedInputTarget-leftSmoothedInput > Constants.MovementConstants.ACCEL_DAMPING_SLOPE) {
//         leftSmoothedInput = leftSmoothedInputTarget;
//       }
//       else {
//         leftSmoothedInput -= Constants.MovementConstants.ACCEL_DAMPING_SLOPE;
//       }
//     }
//     if (rightSmoothedInput < rightSmoothedInputTarget) {
//       if (rightSmoothedInputTarget-rightSmoothedInput < Constants.MovementConstants.ACCEL_DAMPING_SLOPE) {
//         rightSmoothedInput = rightSmoothedInputTarget;
//       }
//       else {
//         rightSmoothedInput += Constants.MovementConstants.ACCEL_DAMPING_SLOPE;
//       }
//     }
//     if (rightSmoothedInput > rightSmoothedInputTarget) {
//       if (rightSmoothedInputTarget-rightSmoothedInput > Constants.MovementConstants.ACCEL_DAMPING_SLOPE) {
//         rightSmoothedInput = rightSmoothedInputTarget;
//       }
//       else {
//         rightSmoothedInput -= Constants.MovementConstants.ACCEL_DAMPING_SLOPE;
//       }
//     }
//     drivetrain.newArcadeDrive(-1*Constants.MovementConstants.LINEAR_SPEED*leftSmoothedInput, Constants.MovementConstants.TURN_SPEED*rightSmoothedInput);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }