// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBackGrabBall extends SequentialCommandGroup {
  /** Creates a new AutoBackGrabBall. */
  public AutoBackGrabBall(Drivetrain m_drive, double endDist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDistance(m_drive, -12, 3000),
      new AutoRotate(m_drive, 135),
      new AutoDistance(m_drive, 120, 5000)
    );
  }
}
