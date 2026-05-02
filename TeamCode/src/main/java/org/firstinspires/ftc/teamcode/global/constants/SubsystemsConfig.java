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
     * Intake motor constants.
     * Responsible for collecting and ejecting game elements.
     */
    public static final class Intake {
        public static final String MOTOR_NAME = "Intake";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final double PULL_POWER = 1.0;
        public static final double PUSH_POWER = -1.0;
        public static final double IDLE_POWER = 0.3;
    }

    /**
     * Indexer motor constants.
     * Responsible for feeding game elements into the flywheel.
     */
    public static final class Indexer {
        public static final String MOTOR_NAME = "Indexer";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final double PULL_POWER = 1.0;
        public static final double PUSH_POWER = -1.0;
        public static final double IDLE_POWER = 0.3;
    }

    /**
     * Stopper servo constants.
     * Responsible for blocking or releasing game elements toward the flywheel.
     */
    public static final class Stopper {
        public static final String SERVO_NAME = "Stopper";
        public static final double OPEN_POSITION = 1.0;
        public static final double CLOSED_POSITION = 0.0;
        public static final double TIME_TO_OPEN_MS = 500;
        public static final double TIME_TO_CLOSE_MS = 500;
    }

    /**
     * Deflector servo constants.
     * Responsible for adjusting the trajectory of game elements.
     */
    public static final class Deflector {
        public static final String SERVO_NAME = "Deflector";
        public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;
        public static final double IDLE_POSITION = 0.0;
    }

    /**
     * LockedTurret servo constants.
     * Debug-only subsystem for locking the turret to a fixed position.
     */
    public static final class LockedTurret {
        public static final String SERVO_NAME = "servo";
        public static final double LOCKED_POSITION = 0.7;
    }

    /**
     * Flywheel motor constants.
     * Responsible for launching game elements at a target RPM.
     */
    public static final class Flywheel {
        public static final String MOTOR_NAME_1 = "FlywheelMotor1";
        public static final String MOTOR_NAME_2 = "FlywheelMotor2";
        public static final double IDLE_POWER = 0.0;
        public static final double TICKS_PER_REV = 28.0;
        public static final double MAX_ACCEL_RPM_PER_SEC = 24000.0;
        public static final double AT_SPEED_TOLERANCE = 50.0;
        public static final double VOLTAGE_ALPHA = 0.01;
        public static final double INITIAL_FILTERED_VOLTAGE = 13.0;
        public static final double KP = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
    }

    /**
     * Turret CRServo constants.
     * Responsible for rotating the turret toward a target angle using a quadratic speed profile.
     */
    public static final class Turret {
        public static final String SERVO_NAME = "Turret";
        public static final String ENCODER_NAME = "TurretEncoder";
        public static final double TICKS_PER_REV = 8192.0;
        public static final double GEAR_RATIO = 5.714;
        public static final double MIN_ANGLE = -80.0;
        public static final double MAX_ANGLE = 80.0;
        public static final double BRAKE_DISTANCE = 50.0;
        public static final double DEAD_ZONE = 3.14;
        public static final double MIN_POWER = 0.07;
        public static final double IDLE_POWER = 0.0;
    }
}