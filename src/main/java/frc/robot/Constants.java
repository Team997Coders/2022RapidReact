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
    }
    
    public static class Controller {
        public static final int CONTROLLER_0 = 0, JOYSTICK_1 = 1, JOYSTICK_2 = 4;
        public static final int CONTROLLER_A = 0, CONTROLLER_X = 1, CONTROLLER_B = 2, CONTROLLER_Y = 3;
        public static final double DEAD_ZONE_SENSITIVITY = 0.2;
    }

    public static class MovementConstants {
        public static final double INPUT_SMOOTH_SLOPE = 0.01;
        public static final double DRIVE_MODIFIER = 0.5;
        public static final double TURN_MODIFIER = 0.5;
    }
}
