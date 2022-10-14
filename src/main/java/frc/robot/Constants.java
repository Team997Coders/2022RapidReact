// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static boolean DEMO_MODE_DEFAULT = true;

    public static double MAX_VOLTAGE_DRAW_VOLTS = 11,

            LOOPTIME_SECS = Robot.kDefaultPeriod,

            MOMENT_KG_PER_M2 = 2.1, // literally no way to test this, and I can't exactly do sysID right now

            MASS_KG = 50; // same

    public static class Ports {

        public static final int FRONT_LEFT_PORT = 13, BACK_LEFT_PORT = 14, FRONT_RIGHT_PORT = 15, BACK_RIGHT_PORT = 16,
                CLIMBER_PORT = 8,
                ZERO_SWITCH_PORT = 0,
                INTAKE_PDP = 11,
                INTAKE_PORT = 12,
                LED_PWM_PORT = 0;
    }

    public static class Controller {

        public static enum JoystickMappingTypes {
            LINEAR,
            QUADRATIC;
        }

        public static final JoystickMappingTypes DEFAULT_MAPPING_TYPE = JoystickMappingTypes.QUADRATIC;

        public static final int CONTROLLER_0 = 0, CONTROLLER_1 = 1, // drive = 0, climb = 1
                JOYSTICK_1 = 1, JOYSTICK_2 = 4, TRIGGER_CLIMB_UP = 2, TRIGGER_CLIMB_DN = 3,
                LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
                A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4,
                UP_ARROW = 5, RIGHT_ARROW = 6, DOWN_ARROW = 7, LEFT_ARROW = 8;

        public static final double DEAD_BAND = 0.04;
    }

    public static class Drive {

        public static enum DriveCommandModes {
            ARCADE_DRIVE,
            CURVATURE_DRIVE,
            MIXED_ARCADE_CURVATURE;
        }

        public static final DriveCommandModes DEFAULT_DRIVE_MODE = DriveCommandModes.ARCADE_DRIVE;

        public static final double MIXED_PROPORTION_ARCADE = 0.2;

        public static final int WHEELS_SPEEDS_MAX_A = 10, WHEEL_SPEEDS_MAX_J = 30,

                AUTO_DISTANCE_MAX_A = 100, AUTO_DISTANCE_MAX_V = 100,

                AUTO_ROTATE_MAX_V = 360, AUTO_ROTATE_MAX_A = 180,

                DRIVE_ENCODER_CPR = 2048;

        public static final double LIN_SLEW_LIMIT = 2, // 1 divided by the number of seconds to go from 0 to 1 speeds
                ROT_SLEW_LIMIT = 4,

                DRIVE_MODIFIER_FULL = 0.5,
                TURN_MODIFIER_FULL = 0.3,

                DRIVE_MODIFIER_DEMO = 0.2,
                TURN_MODIFIER_DEMO = 0.2,

                DRIVE_MODIFIER_SANE = 0.5,
                TURN_MODIFIER_SANE = 0.3,

                RAMSETE_B = 2.1, RAMSETE_ZETA = 0.7,

                WHEEL_SPEEDS_KP = 0.002, WHEEL_SPEEDS_KI = 0, WHEEL_SPEEDS_KD = 0,

                AUTO_DISTANCE_KP = 0.15, AUTO_DISTANCE_KI = 0.05, AUTO_DISTANCE_KD = 0,
                AUTO_DISTANCE_TOL = 3,

                AUTO_ROTATE_KP = 0.012, AUTO_ROTATE_KI = 0.002, AUTO_ROTATE_KD = 0,
                AUTO_ROTATE_TOL = 1,

                DRIVE_TRACK_WIDTH_METERS = 0.35, // 0.546 physically, sysid is being a pain so I can't find empirical
                                                 // (this is a guess)
                DRIVE_WHEEL_DIA_METERS = Units.inchesToMeters(6),
                DRIVE_GEARBOX_RATIO = 7.31,
                DRIVE_METERS_PER_COUNT = (DRIVE_WHEEL_DIA_METERS * Math.PI) / (DRIVE_ENCODER_CPR * DRIVE_GEARBOX_RATIO);
    }

    public static class Climber {

        public static final int CLIMB_FILTER_TAPS = 10,

                CLIMB_LOW_V_LIM = 80, CLIMB_LOW_A_LIM = 80,
                CLIMB_HIGH_V_LIM = 100, CLIMB_HIGH_A_LIM = 200;

        public static final int[] CLIMB_CHECKPOINTS = { 0, 70, 109 };

        public static final double CLIMBER_MAX_HEIGHT = 111,

                CLIMB_LOW_PID_KP = 0.3, CLIMB_LOW_PID_KI = 0.02, CLIMB_LOW_PID_KD = 0,
                CLIMB_HIGH_PID_KP = 0.4, CLIMB_HIGH_PID_KI = 0.02, CLIMB_HIGH_PID_KD = 0,
                CLIMB_SETPOINT_TOLERANCE = 3,

                CURRENT_THRESHOLD = 30;

    }

    public static class Intake {
        public static final double INTAKE_DEFAULT_SPEED = 0.75;
    }

    public static class Lighting {

        public static final int LED_COUNT = 17;

        public static final double DEFAULT_ALTERNATING_TIME_S = 500;
    }
}
