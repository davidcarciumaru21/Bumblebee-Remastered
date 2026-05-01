package org.firstinspires.ftc.teamcode.global.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        public static final String MOTOR_NAME                 = "Intake";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final double PULL_POWER                 = 1.0;
        public static final double PUSH_POWER                 = -1.0;
        public static final double IDLE_POWER                 = 0.0;
    }

    /**
     * Indexer motor constants.
     * Responsible for feeding game elements into the flywheel.
     */
    public static final class Indexer {
        public static final String MOTOR_NAME                 = "Indexer";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final double PULL_POWER                 = 1.0;
        public static final double PUSH_POWER                 = -1.0;
        public static final double IDLE_POWER                 = 0.0;
    }

    /**
     * Stopper servo constants.
     * Responsible for blocking or releasing game elements toward the flywheel.
     */
    public static final class Stopper {
        public static final String SERVO_NAME       = "Stopper";
        public static final double OPEN_POSITION    = 1.0;
        public static final double CLOSED_POSITION  = 0.0;
        public static final double TIME_TO_OPEN_MS  = 500;
        public static final double TIME_TO_CLOSE_MS = 500;
    }
}