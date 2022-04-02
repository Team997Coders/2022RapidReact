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

        public static final int 
            FRONT_LEFT_PORT = 13, BACK_LEFT_PORT = 14, FRONT_RIGHT_PORT = 15, BACK_RIGHT_PORT = 16,
            CLIMBER_PORT = 8,
            INTAKE_PORT = 5, //just making this up
            ZERO_SWITCH_PORT = 0,
            LED_PWM_PORT = 0;
    }
    public static class Controller {

        public static final int 
            CONTROLLER_0 = 0, CONTROLLER_1 = 1, // drive = 0, climb = 1
            JOYSTICK_1 = 1, JOYSTICK_2 = 4, TRIGGER_CLIMB_UP = 2, TRIGGER_CLIMB_DN = 3,
            LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
            A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4,
            UP_ARROW = 5, RIGHT_ARROW = 6, DOWN_ARROW = 7, LEFT_ARROW = 8; // check
        
        public static final double DEAD_ZONE_SENSITIVITY = 0.02;
    }

    public static class Drive {

        public static final double 
            INPUT_SMOOTH_SLOPE = 0.03,
            DRIVE_MODIFIER = 1.0,
            TURN_MODIFIER = 0.5,

            AUTO_DISTANCE_KP = 0.1, AUTO_DISTANCE_KI = 0, AUTO_DISTANCE_KD = 0,
            AUTO_DISTANCE_MAX_V = 200, AUTO_DISTANCE_MAX_A = 100,
            AUTO_DISTANCE_TOL = 0.01,

            AUTO_ROTATE_KP = 0.005, AUTO_ROTATE_KI = 0, AUTO_ROTATE_KD = 0,
            AUTO_ROTATE_MAX_V = 360, AUTO_ROTATE_MAX_A = 60,
            AUTO_ROTATE_TOL = 0.01,

            DRIVE_ENCODER_CPR = 2048,
            DRIVE_WHEEL_DIA_IN = 6,
            DRIVE_GEARBOX_RATIO = 7.31,
            DRIVE_IN_PER_COUNT = (DRIVE_WHEEL_DIA_IN * Math.PI) / (DRIVE_ENCODER_CPR * DRIVE_GEARBOX_RATIO);
    }

    public static class Climber {

        public static final int CLIMBER_MAX_HEIGHT = 109;
    }

    public static class Intake {
        public static final double INTAKE_DEFAULT_SPEED = 0.3; // idk
    }
    
    public static class Lighting {
        
        public static final int 
            LED_COUNT = 17,
            DEFAULT_ALTERNATING_TIME_MS = 500;
    }
}
