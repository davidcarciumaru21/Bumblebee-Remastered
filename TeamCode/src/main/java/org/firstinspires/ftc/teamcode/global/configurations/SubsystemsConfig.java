package org.firstinspires.ftc.teamcode.global.configurations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.pedropathing.ftc.localization.Encoder;

/**
 * Hardware constants for all subsystems.
 * Each inner class corresponds to one subsystem.
 * Modify values here to tune robot behavior without touching subsystem logic.
 */
public class SubsystemsConfig {

    /**
     * Voltage sensor constants.
     * Used for filtered voltage readings across tuners and subsystems.
     * INITIAL_FILTERED_VOLTAGE — starting value for the exponential filter, should approximate battery voltage.
     * VOLTAGE_ALPHA — filter coefficient between 0 and 1. Lower = smoother but slower to react.
     */
    public static final class VoltageSensor {
        public static final double INITIAL_FILTERED_VOLTAGE = 13.0;
        public static final double VOLTAGE_ALPHA            = 0.01;
    }

    /**
     * Intake motor constants.
     * Responsible for collecting and ejecting game elements.
     * PULL_POWER         — power applied when collecting. Positive = inward.
     * PUSH_POWER         — power applied when ejecting. Negative = outward.
     * IDLE_POWER         — power applied when idle. Non-zero to hold game elements in place.
     * STALL_CURRENT_AMPS — current threshold in amps above which stall is detected.
     * STALL_TIME_MS      — time in ms current must exceed threshold before stall is confirmed.
     */
    public static final class Intake {
        public static final String                  MOTOR_NAME         = "Intake";
        public static final DcMotorSimple.Direction DIRECTION          = DcMotorSimple.Direction.FORWARD;
        public static final double                  PULL_POWER         = 1.0;
        public static final double                  PUSH_POWER         = -1.0;
        public static final double                  IDLE_POWER         = 0.4;
        public static final double                  STALL_CURRENT_AMPS = 6.0;
        public static final double                  STALL_TIME_MS      = 300.0;
    }

    /**
     * Indexer motor constants.
     * Responsible for feeding game elements into the flywheel.
     * PULL_POWER — power applied when feeding toward flywheel. Positive = toward flywheel.
     * PUSH_POWER — power applied when reversing. Negative = away from flywheel.
     * IDLE_POWER — power applied when idle. Non-zero to hold game elements in place.
     */
    public static final class Indexer {
        public static final String                  MOTOR_NAME = "Indexer";
        public static final DcMotorSimple.Direction DIRECTION  = DcMotorSimple.Direction.FORWARD;
        public static final double                  PULL_POWER = 1.0;
        public static final double                  PUSH_POWER = -1.0;
        public static final double                  IDLE_POWER = 0.4;
    }

    /**
     * Stopper servo constants.
     * Responsible for blocking or releasing game elements toward the flywheel.
     * OPEN_POSITION    — servo position when stopper is fully open.
     * CLOSED_POSITION  — servo position when stopper is fully closed.
     * TIME_TO_OPEN_MS  — time in ms to wait before considering stopper fully open.
     * TIME_TO_CLOSE_MS — time in ms to wait before considering stopper fully closed.
     */
    public static final class Stopper {
        public static final String SERVO_NAME       = "Stopper";
        public static final double OPEN_POSITION    = 0.15;
        public static final double CLOSED_POSITION  = 0.625;
        public static final double TIME_TO_OPEN_MS  = 150;
        public static final double TIME_TO_CLOSE_MS = 150;
    }

    /**
     * Deflector servo constants.
     * Responsible for adjusting the trajectory of game elements.
     * IDLE_POSITION  — servo position when deflector is at rest.
     * OPEN_POSITION  — servo position when deflector is fully open.
     * CLOSE_POSITION — servo position when deflector is fully closed.
     * MIN_ANGLE      — minimum launch angle in radians (relative to horizontal). Measured physically.
     * MAX_ANGLE      — maximum launch angle in radians (relative to horizontal). Measured physically.
     */
    public static final class Deflector {
        public static final String          SERVO_NAME     = "Deflector";
        public static final Servo.Direction DIRECTION      = Servo.Direction.FORWARD;
        public static final double          IDLE_POSITION  = 0.55;
        public static final double          MIN_ANGLE      = Math.toRadians(50);
        public static final double          MAX_ANGLE      = Math.toRadians(70);
    }

    /**
     * LockedTurret servo constants.
     * Debug-only subsystem for locking the turret to a fixed position.
     * LOCKED_POSITION — servo position to lock the turret to. Tune with LockedTurretTuner.
     */
    public static final class LockedTurret {
        public static final String SERVO_NAME      = "servo";
        public static final double LOCKED_POSITION = 0.7;
    }

    /**
     * Flywheel motor constants.
     * Responsible for launching game elements at a target RPM.
     * IDLE_POWER            — power applied when idle. Non-zero keeps flywheel spinning slowly.
     * TICKS_PER_REV         — encoder ticks per motor revolution. Specific to motor model.
     * MAX_ACCEL_RPM_PER_SEC — maximum RPM change per second. Limits ramp rate.
     * AT_SPEED_TOLERANCE    — RPM tolerance to consider flywheel at target speed.
     * KP                    — proportional gain. Tune with FlywheelKpTuner.
     * KS                    — static feedforward in volts. Tune with FlywheelKsTuner.
     * KV                    — velocity feedforward in V/RPM. Tune with FlywheelKvTuner.
     */
    public static final class Flywheel {
        public static final String MOTOR_NAME_1          = "FlywheelMotor1";
        public static final String MOTOR_NAME_2          = "FlywheelMotor2";
        public static final double IDLE_POWER            = 0.0;
        public static final double TICKS_PER_REV         = 28.0;
        public static final double MAX_ACCEL_RPM_PER_SEC = 24000.0;
        public static final double AT_SPEED_TOLERANCE    = 80.0;
        public static final double KP                    = 0.023;
        public static final double KS                    = 0.09516;
        public static final double KV                    = 0.0024;
    }

    /**
     * Turret CRServo constants.
     * Responsible for rotating the turret toward a target angle using a quadratic speed profile.
     * TICKS_PER_REV   — encoder ticks per revolution. Specific to encoder model.
     * GEAR_RATIO      — gear ratio between encoder and turret output shaft.
     * MIN_ANGLE       — minimum allowed turret angle in degrees.
     * MAX_ANGLE       — maximum allowed turret angle in degrees.
     * BRAKE_DISTANCE  — angular distance in degrees at which deceleration begins.
     * DEAD_ZONE       — angular error in degrees below which turret is considered at target.
     * MIN_POWER_VOLTS — minimum voltage needed to overcome static friction. Tune with TurretMinPowerTuner.
     * IDLE_POWER      — power applied when turret is idle.
     */
    public static final class Turret {
        public static final String SERVO_NAME = "Turret";
        public static final String ENCODER_NAME = "FrontRight";
        public static final double TICKS_PER_REV = 8192.0;
        public static final double GEAR_RATIO = 5.714;
        public static final double MIN_ANGLE = -90.0;
        public static final double MAX_ANGLE = 70.0;
        public static final double BRAKE_DISTANCE = 100.0;
        public static final double DEAD_ZONE = 1.0;
        public static final double MIN_POWER_VOLTS = 0.90;
        public static final double IDLE_POWER = -1.0;
    }

    /**
     *
     * Drivetrain motor constants.
     * Used by both ManualTeleOp and Pedro Pathing.
     * LEFT_FRONT_MOTOR_NAME       — hardware map name for the front left motor.
     * LEFT_REAR_MOTOR_NAME        — hardware map name for the rear left motor.
     * RIGHT_FRONT_MOTOR_NAME      — hardware map name for the front right motor.
     * RIGHT_REAR_MOTOR_NAME       — hardware map name for the rear right motor.
     * LEFT_FRONT_MOTOR_DIRECTION  — direction of the front left motor.
     * LEFT_REAR_MOTOR_DIRECTION   — direction of the rear left motor.
     * RIGHT_FRONT_MOTOR_DIRECTION — direction of the front right motor.
     * RIGHT_REAR_MOTOR_DIRECTION  — direction of the rear right motor.
     * X_VELOCITY                  — maximum velocity in the x direction in inches/second. Tune with Pedro Pathing.
     * Y_VELOCITY                  — maximum velocity in the y direction in inches/second. Tune with Pedro Pathing.
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
    }

    /**
     * Localizer constants for Pedro Pathing.
     * Uses three dead wheel odometry pods and IMU.
     * LEFT_POD_Y                  — y offset of the left parallel pod in inches relative to robot center.
     * RIGHT_POD_Y                 — y offset of the right parallel pod in inches relative to robot center.
     * STRAFE_POD_X                — x offset of the strafe pod in inches relative to robot center.
     * LEFT_ENCODER_NAME           — hardware map name of the motor port the left encoder is plugged into.
     * RIGHT_ENCODER_NAME          — hardware map name of the motor port the right encoder is plugged into.
     * STRAFE_ENCODER_NAME         — hardware map name of the motor port the strafe encoder is plugged into.
     * LEFT_ENCODER_DIRECTION      — encoder direction for the left pod. Reverse if x decreases when moving forward.
     * RIGHT_ENCODER_DIRECTION     — encoder direction for the right pod. Reverse if x decreases when moving forward.
     * STRAFE_ENCODER_DIRECTION    — encoder direction for the strafe pod. Reverse if y decreases when moving left.
     * FORWARD_TICKS_TO_INCHES     — multiplier converting encoder ticks to inches for forward movement. Tune with forward tuner.
     * STRAFE_TICKS_TO_INCHES      — multiplier converting encoder ticks to inches for lateral movement. Tune with lateral tuner.
     * TURN_TICKS_TO_INCHES        — multiplier converting encoder ticks to inches for rotation. Tune with turn tuner.
     * IMU_HARDWARE_MAP_NAME       — hardware map name of the IMU.
     * IMU_ORIENTATION             — orientation of the Control Hub on the robot.
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
}