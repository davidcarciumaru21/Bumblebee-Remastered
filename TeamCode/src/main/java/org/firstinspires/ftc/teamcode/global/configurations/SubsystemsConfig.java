package org.firstinspires.ftc.teamcode.global.configurations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.pedropathing.ftc.localization.Encoder;

/**
 * Configuration registry for all physical hardware components and constants on the robot.
 * Each inner class isolates properties of a specific subsystem or functional module.
 * Centralizing these values allows the drive team to rapidly modify and fine-tune physical behaviors
 * (such as motor power, servo targets, tolerances, and software multipliers) without altering system execution code.
 */
public class SubsystemsConfig {

    /**
     * Voltage sensor configuration.
     * Essential for stabilizing subsystem performance across varying battery discharge curves.
     * INITIAL_FILTERED_VOLTAGE — The initial baseline estimate used to seed the moving average filter on startup.
     * VOLTAGE_ALPHA            — Low-pass filter coefficient (0 to 1). Determines how heavily new raw readings impact the smooth average.
     */
    public static final class VoltageSensor {
        public static final double INITIAL_FILTERED_VOLTAGE = 13.0;
        public static final double VOLTAGE_ALPHA            = 0.01;
    }

    /**
     * Active intake mechanism constants.
     * Manages raw intake execution power parameters and advanced stall protection thresholds.
     * MOTOR_NAME         — Configuration name matching the entry inside the REV Hardware Map.
     * DIRECTION          — Rotational direction required to spin the collection rollers inward.
     * PULL_POWER         — Active collection power factor applied during teleoperated intake sequences.
     * PUSH_POWER         — Active expulsion power factor applied to forcefully spit out trapped elements.
     * IDLE_POWER         — Minimal passive holding power applied to maintain element grip inside the mechanism tray.
     * STALL_CURRENT_AMPS — Current safety margin in Amps. Readings above this indicate a physical jam.
     * STALL_TIME_MS      — Time buffer in milliseconds that a current spike must persist to trigger automatic software unjamming.
     */
    public static final class Intake {
        public static final String                  MOTOR_NAME         = "Intake";
        public static final DcMotorSimple.Direction DIRECTION          = DcMotorSimple.Direction.FORWARD;
        public static final double                  PULL_POWER         = 1.0;
        public static final double                  PUSH_POWER         = -1.0;
        public static final double                  IDLE_POWER         = 0.3;
        public static final String                  CURRENT_SENSOR_NAME= "Intake";
    }

    /**
     * Internal indexing system constants.
     * Controls the secondary serial pathway that channels elements from the entry tray directly up to the flywheel throat.
     * MOTOR_NAME — Configuration identifier inside the control hub's hardware map layout.
     * DIRECTION  — Direction setup mapped to push elements upstream toward the shooter mechanism.
     * PULL_POWER — Forward feed power factor applied when actively indexing elements into the launching zone.
     * PUSH_POWER — Reverse power factor used during systemic clearance or purge protocols.
     * IDLE_POWER — Minor constant force applied to ensure elements do not bounce loosely out of queueing lines.
     */
    public static final class Indexer {
        public static final String                  MOTOR_NAME = "Indexer";
        public static final DcMotorSimple.Direction DIRECTION  = DcMotorSimple.Direction.FORWARD;
        public static final double                  PULL_POWER = 1.0;
        public static final double                  PUSH_POWER = -1.0;
        public static final double                  IDLE_POWER = 0.2;
    }

    /**
     * Mechanical element gate (Stopper) configurations.
     * Fast-acting physical barrier preventing elements from rolling into a spinning flywheel prematurely.
     * SERVO_NAME       — The target name configured for this physical servo inside the REV Hub software layout.
     * OPEN_POSITION    — Absolute servo position scalar representing a cleared and unrestricted launch channel.
     * CLOSED_POSITION  — Absolute servo position scalar representing a completely blocked and safely locked channel.
     * TIME_TO_OPEN_MS  — Estimated mechanical sweep time required for the physical armature to clear the channel.
     * TIME_TO_CLOSE_MS — Estimated mechanical sweep time required for the physical armature to re-engage the channel barrier.
     */
    public static final class Stopper {
        public static final String SERVO_NAME       = "Stopper";
        public static final double OPEN_POSITION    = 0.15;
        public static final double CLOSED_POSITION  = 0.625;
        public static final double TIME_TO_OPEN_MS  = 250;
        public static final double TIME_TO_CLOSE_MS = 250;
    }

    /**
     * Variable trajectory exit ramp (Deflector) constants.
     * Dynamically scales the vertical launch angle profile based on absolute field-relative distance tracking.
     * SERVO_NAME     — Hardware mapping name for the physical deflector servo actuator.
     * DIRECTION      — Physical sweep orientation matching increasing trajectory angles.
     * IDLE_POSITION  — Rest target position when no active firing calculations are requested by the primary scheduler.
     * MIN_ANGLE      — Bottom-range mechanical bound measured in radians relative to the flat chassis floor.
     * MAX_ANGLE      — Upper mechanical safety limit measured in radians relative to the flat chassis floor.
     */
    public static final class Deflector {
        public static final String          SERVO_NAME     = "Deflector";
        public static final Servo.Direction DIRECTION      = Servo.Direction.FORWARD;
        public static final double          IDLE_POSITION  = 0.55;
        public static final double          MIN_ANGLE      = Math.toRadians(50);
        public static final double          MAX_ANGLE      = Math.toRadians(70);
    }

    /**
     * Static debug turret lock properties.
     * Primarily used within isolated tuning environments to hold the structural platform perfectly rigid.
     * LOCKED_POSITION — Absolute servo position target required to clamp the mechanism safely against rotation.
     */
    public static final class LockedTurret {
        public static final String SERVO_NAME      = "servo";
        public static final double LOCKED_POSITION = 0.0;
    }

    /**
     * High-speed dual flywheel launcher constants.
     * Manages velocity acceleration limits, velocity tracking, and feedforward tuning vectors.
     * MOTOR_NAME_1 / 2      — Matching hardware configuration entries for both paired drive motors.
     * IDLE_POWER            — Constant base power modifier used to keep the rotors idling at low energy between active shots.
     * IDLE_RPM              — Closed-loop flywheel speed held while no shot is actively requested.
     * TICKS_PER_REV         — Resolution count of the underlying digital optical encoder system.
     * MAX_ACCEL_RPM_PER_SEC — Safety limit restricting acceleration changes to avoid breaking gears or tearing high-speed belts.
     * AT_SPEED_TOLERANCE    — Maximum RPM error variance allowed before system signals "ready-to-fire" to the indexer.
     * KP                    — Proportional tuning coefficient. Directly counteracts instant error.
     * KS                    — Feedforward component used to break initial motor shaft friction and overcome cogging.
     * KV                    — Velocity feedforward scalar converting structural target RPM directly to voltage outputs.
     */
    public static final class Flywheel {
        public static final String MOTOR_NAME_1          = "FlywheelMotor1";
        public static final String MOTOR_NAME_2          = "FlywheelMotor2";
        public static final double IDLE_POWER            = 0.0;
        public static final double IDLE_RPM              = 2400.0;
        public static final double TICKS_PER_REV         = 28.0;
        public static final double THREE_BALL_SHOT_TIME_MS = 900.0;
        public static final double MAX_ACCEL_RPM_PER_SEC = 24000.0;
        public static final double AT_SPEED_TOLERANCE    = 80.0;
        public static final double KP                    = 0.02;
        public static final double KS                    = 0.057;
        public static final double KV                    = 0.0022;
    }

    /**
     * Continuous tracking rotational turret constants.
     * Operates a multi-turn proportional servo assembly linked via external reduction gears to achieve field-centric isolation.
     * SERVO_NAME    — Name parameter utilized to map the physical servo connection on the REV expansion hub.
     * MIN_ANGLE     — Structural safe travel threshold bounded in degrees to prevent umbilical wrap damage.
     * MAX_ANGLE     — Opposite directional structural threshold bounded in degrees to prevent physical collision.
     * IDLE_POSITION — Hardware home target mapping to 0 degrees relative to the direct centerline of the drive chassis.
     */
    public static final class Turret {
        public static final String SERVO_NAME                   = "Turret";
        public static final String ENCODER_NAME                 = "FrontRight";
        public static final double MIN_ANGLE                    = -45.0;
        public static final double MAX_ANGLE                    = 45.0;
        public static final double IDLE_POSITION                = 0.5;
        public static final double ENCODER_TICKS_PER_DEGREE     = 22.76 * 5.71;
        public static final double ENCODER_KP                   = 0.05;
        public static final double MAX_SERVO_POSITION_RATE      = 1;
        public static final double ANGLE_TOLERANCE_DEGREES      = 0.5;
    }

    /**
     * Mecanum drivetrain chassis constants.
     * Provides physical configuration maps for base teleop driving and specialized driver scaling modes.
     * FRONT/BACK MOTOR NAMES — Hardware designators mapped to the exact directional corners of the physical frame.
     * X_VELOCITY / Y_VELOCITY— Velocity empirical feedforward tuning modifiers utilized by Pedro Pathing.
     * SLOW_MODE_COEFFICIENT — Linear speed scale modifier applied to full motor throttle when precision driver manipulation is active.
     */
    public static final class Drivetrain {
        public static final String                  FRONT_LEFT_MOTOR_NAME       = "FrontLeft";
        public static final String                  BACK_LEFT_MOTOR_NAME        = "BackLeft";
        public static final String                  FRONT_RIGHT_MOTOR_NAME      = "FrontRight";
        public static final String                  BACK_RIGHT_MOTOR_NAME       = "BackRight";
        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION  = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION   = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION  = DcMotorSimple.Direction.FORWARD;
        public static final double                  X_VELOCITY                  = 79.7768;
        public static final double                  Y_VELOCITY                  = 64.4725;
        public static final double                  MAX_POWER                   = 1.0;
        public static final double                  SLOW_MODE_COEFFICIENT       = 0.40;
    }

    /**
     * Absolute physical robot frame layout dimensions.
     * Values are stored in raw real-world inches to handle dynamic wall relocalization.
     * LENGTH — Distance across the front-to-back chassis footprint.
     * WIDTH  — Distance across the left-to-right side-to-side footprint.
     */
    public static final class RobotDimensions {
        public static final double LENGTH = 17.20;
        public static final double WIDTH  = 14.33;
        public static final double TurreToCenterDistance = 0;
    }

    /**
     * High-precision dead wheel odometry localizer configuration for Pedro Pathing.
     * Outlines layout geometries, ticks-to-inches scalars, and hardware orientations.
     * POD_X / Y POSITIONS — Displacement offsets measured relative to the geometric center point of the robot chassis.
     * FORWARD / STRAFE / TURN INCHES — Calibration conversions matching raw encoder tick transitions to actual linear inches traveled.
     */
    public static final class Localizer {
        public static final double LEFT_POD_Y              = 4.25;
        public static final double RIGHT_POD_Y             = -4.25;
        public static final double STRAFE_POD_X            = -2.93;

        public static final String LEFT_ENCODER_NAME       = "BackRight";
        public static final String RIGHT_ENCODER_NAME      = "Indexer";
        public static final String STRAFE_ENCODER_NAME     = "Intake";

        public static final double LEFT_ENCODER_DIRECTION   = Encoder.FORWARD;
        public static final double RIGHT_ENCODER_DIRECTION  = Encoder.FORWARD;
        public static final double STRAFE_ENCODER_DIRECTION = Encoder.FORWARD;

        public static final double FORWARD_TICKS_TO_INCHES = -0.001966;
        public static final double STRAFE_TICKS_TO_INCHES  = -0.001988;
        public static final double TURN_TICKS_TO_INCHES    = -0.001996;

        public static final String IMU_HARDWARE_MAP_NAME   = "imu";
        public static final RevHubOrientationOnRobot IMU_ORIENTATION = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }

    /**
     * Limelight camera configuration.
     * Used for AprilTag/global-pose relocalization into Pedro Pathing field coordinates.
     */
    public static final class Limelight {
        public static final String HARDWARE_MAP_NAME = "limelight";
        public static final int DEFAULT_PIPELINE = 0;
        public static final int GREEN_BALL_PIPELINE = 8;
        public static final int PURPLE_BALL_PIPELINE = 8;
        public static final double METERS_TO_INCHES = 39.3701;
        public static final double PEDRO_FIELD_CENTER_OFFSET_INCHES = 72.0;
        public static final double MAX_VALID_Z_ERROR_INCHES = 6.0;
        public static final double FILTER_ALPHA = 0.25;

        public static final class Maze {
            public static final int ROW_COUNT = 3;
            public static final int COLUMN_COUNT = 3;
            public static final int GREEN_ROW_COUNT = 2;
            
            public static final double ORIGIN_X_INCHES = 47.2;
            public static final double ORIGIN_Y_INCHES = 23.5;
            public static final double ROW_SPACING_INCHES = 36.0;
            public static final double COLUMN_SPACING_INCHES = 18.0;
            public static final double SCAN_FRONT_CLEARANCE_INCHES = 10.0;
            public static final double PASSED_ROW_REAR_CLEARANCE_INCHES = 14.0;
            public static final double SCAN_X_OFFSET_INCHES =
                    RobotDimensions.LENGTH / 2.0 + SCAN_FRONT_CLEARANCE_INCHES;
            public static final double HEADING_DEGREES = 90.0;

            public static final double START_X_INCHES = ORIGIN_X_INCHES
                    - SCAN_X_OFFSET_INCHES * Math.cos(Math.toRadians(HEADING_DEGREES));
            public static final double START_Y_INCHES = ORIGIN_Y_INCHES
                    - SCAN_X_OFFSET_INCHES * Math.sin(Math.toRadians(HEADING_DEGREES));
            public static final double START_HEADING_DEGREES = HEADING_DEGREES;

            public static final boolean USE_POSE_BASED_COLUMN_TX = false;
            public static final double[] COLUMN_X_DEGREES = {-16.0, 0.0, 16.0};
            public static final double FIELD_BEARING_TO_TX_SIGN = -1.0;
            public static final double COLUMN_X_TOLERANCE_DEGREES = 8.0;
            public static final double SCAN_TARGET_X_DEGREES = 0.0;
            public static final double SCAN_TARGET_X_TOLERANCE_DEGREES = 8.0;
            public static final double MIN_TARGET_AREA = 0.01;

            public static final double PIPELINE_SETTLE_MS = 180.0;
            public static final double ROW_SCAN_TIME_MS = 350.0;
            public static final double COLUMN_SCAN_TIME_MS = 350.0;
            public static final double MAX_ROW_SCAN_TIME_MS = 1400.0;
            public static final boolean ALLOW_UNRELIABLE_FALLBACK = true;
            public static final int FALLBACK_COLUMN_INDEX = 1;
            public static final double PATH_TIMEOUT_MS = 5000.0;
            public static final double PATH_MAX_POWER = 0.65;
            public static final double LEARNED_PATH_MAX_POWER = 1.0;
            public static final double BASE_COLLECTION_TIME_MS = 1000.0;
            public static final double SHOOT_START_TIMEOUT_MS = 6000.0;
            public static final double SHOOT_STOP_TIMEOUT_MS = 2000.0;
        }
    }
}
