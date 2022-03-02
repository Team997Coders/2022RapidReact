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
        public static final int CLIMBER_PORT = 8;
        public static final int ZERO_SWITCH_PORT = 0;
    }
    public static class Controller {
        public static final int CONTROLLER_0 = 0;
        public static final int JOYSTICK_1 = 1, JOYSTICK_2 = 4, TRIGGER_CLIMB_UP = 3, TRIGGER_CLIMB_DN = 2;
        public static final int A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4;
        public static final double DEAD_ZONE_SENSITIVITY = 0.05;
    }
    
    public static final int CLIMBER_MAX_HEIGHT = 2000; // temporary until physical measurements can be made

    public static class MovementConstants {
        public static final double INPUT_SMOOTH_SLOPE_NORMAL = 0.02;
        public static final double DRIVE_MODIFIER = -1;
        public static final double TURN_MODIFIER = -0.5;
    }

    public static final double DRIVE_ENCODER_CPR = 2048;
    public static final double DRIVE_WHEEL_DIA_IN = 6;
    public static final double DRIVE_GEARBOX_RATIO = 7.31;
    public static final double DRIVE_IN_PER_COUNT = (DRIVE_WHEEL_DIA_IN*Math.PI) / (DRIVE_ENCODER_CPR*DRIVE_GEARBOX_RATIO);
}
