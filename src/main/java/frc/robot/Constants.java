// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Ports {
        
        public static final int[] DRIVE_PORTS = {13, 14, 15, 16};
        public static final int CONTROLLER_0 = 0, CONTROLLER_1 = 1, JOYSTICK_1 = 1, JOYSTICK_2 = 4;
        public static final int CLIMBER_MOTOR_PORT = 9;
        public static double SPEED = 0.1;
        public static int MAXIMUM_EXTENSION = 100;
        public static int MINIMUM_EXTENSION = 0;
    }
}
