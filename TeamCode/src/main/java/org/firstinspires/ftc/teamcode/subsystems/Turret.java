package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.subsystemsEnums.TurretState;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * Controls the turret servo using a linear regression to convert angle to servo position.
 * The servo supports 5 full turns, allowing precise positioning.
 *
 * <p>Regression: position = SCALE * angle + OFFSET
 * Tune SCALE and OFFSET with TurretTuner.
 *
 * State machine:
 * - IDLE        — turret holds at center position (0.5)
 * - AT_POSITION — turret holds at a custom target angle
 */
public class Turret implements Subsystem {

    private final Servo turret;

    private TurretState state          = TurretState.IDLE;
    private double      targetPosition = 0.5;

    public Turret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(Servo.class, SubsystemsConfig.Turret.SERVO_NAME);
        this.turret.setPosition(0.5);
    }

    /**
     * Sets the target angle in degrees and transitions to AT_POSITION.
     * Angle is clamped to the configured min/max range.
     * @param angle target angle in degrees
     */
    public void setTargetAngle(double angle) {
        angle = Math.max(
                SubsystemsConfig.Turret.MIN_ANGLE,
                Math.min(SubsystemsConfig.Turret.MAX_ANGLE, angle)
        );
        this.targetPosition = MathUtils.clamp(
                0.00000123457 * angle * angle + 0.00166667 * angle + 0.5,
                0.0,
                1.0
        );
        this.state = TurretState.AT_POSITION;
    }

    /** Returns turret to center position (0.5). */
    public void idle() {
        this.targetPosition = 0.5;
        this.state          = TurretState.IDLE;
    }

    /** Returns true if turret is at a custom position. */
    public boolean isAtPosition() { return this.state == TurretState.AT_POSITION; }

    /** Returns the current state of the turret. */
    public TurretState getState() { return this.state; }

    /** Returns the current target servo position. */
    public double getTargetPosition() { return this.targetPosition; }

    /** Applies the staged position to hardware. Must be called every loop. */
    @Override
    public void update() {
        this.turret.setPosition(this.targetPosition);
    }
}