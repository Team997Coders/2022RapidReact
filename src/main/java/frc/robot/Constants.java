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
        public static final int CLIMBER_PORT = 9;
        public static final int CLIMBER_ZERO_SWITCH_PORT = 0;
        public static final int MAGNET_SWITCH_TEST_PORT = 5; // idk
    }
    public static class Controller {
        public static final int CONTROLLER_1 = 0;
        public static final int JOYSTICK_1 = 1, JOYSTICK_2 = 4;
        public static final int A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4;
        public static final double DEAD_ZONE_SENSITIVITY = 0.1;
    }
    public static class MovementConstants {
        public static final double INPUT_SMOOTH_SLOPE = 0.1;
        public static class ClimberConstants {
            public static final int CLIMBER_MAX_HEIGHT = 100; // temporary until physical measurements can be made
            public static final double CLIMBER_KP = 0, CLIMBER_KI = 0, CLIMBER_KD = 0; // temporary until tuning is done
            public static final double CLIMB_CONSTRAINT_ACCEL = 1, CLIMB_CONSTRAINT_V = 1; // tune
        }
        public static class DrivetrainConstants {
            public static final double DRIVE_LIN_KP = 0, DRIVE_LIN_KI = 0, DRIVE_LIN_KD = 0; // similarly temporary
            public static final double DRIVE_LIN_CONSTRAINT_ACCEL = 1, DRIVE_LIN_CONSTRAINT_V = 1; // tune 
            public static final double DRIVE_ROT_KP = 0, DRIVE_ROT_KI = 0, DRIVE_ROT_KD = 0; // similarly temporary
            public static final double DRIVE_ROT_CONSTRAINT_ACCEL = 1, DRIVE_ROT_CONSTRAINT_V = 1; // tune
            public static final double ENCODER_TO_DISTANCE_FACTOR = 1; // to replace for obvious reasons
        }
    }
}
