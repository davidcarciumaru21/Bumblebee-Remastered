package org.firstinspires.ftc.teamcode.global.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
     * PULL_POWER — power applied when collecting. Positive = inward.
     * PUSH_POWER — power applied when ejecting. Negative = outward.
     * IDLE_POWER — power applied when idle. Non-zero to hold game elements in place.
     */
    public static final class Intake {
        public static final String                  MOTOR_NAME = "Intake";
        public static final DcMotorSimple.Direction DIRECTION  = DcMotorSimple.Direction.FORWARD;
        public static final double                  PULL_POWER = 1.0;
        public static final double                  PUSH_POWER = -1.0;
        public static final double                  IDLE_POWER = 0.3;
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
        public static final DcMotorSimple.Direction DIRECTION  = DcMotorSimple.Direction.REVERSE;
        public static final double                  PULL_POWER = 1.0;
        public static final double                  PUSH_POWER = -1.0;
        public static final double                  IDLE_POWER = 0.3;
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
        public static final double OPEN_POSITION    = 1.0;
        public static final double CLOSED_POSITION  = 0.0;
        public static final double TIME_TO_OPEN_MS  = 500;
        public static final double TIME_TO_CLOSE_MS = 500;
    }

    /**
     * Deflector servo constants.
     * Responsible for adjusting the trajectory of game elements.
     * IDLE_POSITION              — servo position when deflector is at rest.
     * DEGREES_TO_POSITION_SCALE  — slope of the linear mapping from degrees to servo position.
     * DEGREES_TO_POSITION_OFFSET — intercept of the linear mapping from degrees to servo position.
     */
    public static final class Deflector {
        public static final String          SERVO_NAME                 = "Deflector";
        public static final Servo.Direction DIRECTION                  = Servo.Direction.FORWARD;
        public static final double          IDLE_POSITION              = 0.0;
        public static final double          DEGREES_TO_POSITION_SCALE  = 0.0;
        public static final double          DEGREES_TO_POSITION_OFFSET = 0.0;
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
        public static final double AT_SPEED_TOLERANCE    = 50.0;
        public static final double KP                    = 0.0;
        public static final double KS                    = 0.0;
        public static final double KV                    = 0.0;
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
        public static final String SERVO_NAME      = "Turret";
        public static final String ENCODER_NAME    = "TurretEncoder";
        public static final double TICKS_PER_REV   = 8192.0;
        public static final double GEAR_RATIO      = 5.714;
        public static final double MIN_ANGLE       = -80.0;
        public static final double MAX_ANGLE       = 80.0;
        public static final double BRAKE_DISTANCE  = 50.0;
        public static final double DEAD_ZONE       = 3.14;
        public static final double MIN_POWER_VOLTS = 0.0;
        public static final double IDLE_POWER      = 0.0;
    }
}