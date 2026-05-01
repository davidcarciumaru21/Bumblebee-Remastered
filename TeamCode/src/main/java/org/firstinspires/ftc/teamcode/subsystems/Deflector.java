package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.DeflectorState;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * Controls the deflector servo responsible for adjusting the trajectory of game elements.
 * Position is staged internally and applied to hardware only in {@link #update()}.
 */
public class Deflector implements Subsystem {

    private final Servo deflector;
    private DeflectorState state = DeflectorState.IDLE;
    private double targetPosition = SubsystemsConfig.Deflector.IDLE_POSITION;

    public Deflector(HardwareMap hardwareMap) {
        this.deflector = hardwareMap.get(Servo.class, SubsystemsConfig.Deflector.SERVO_NAME);
        this.deflector.setDirection(SubsystemsConfig.Deflector.DIRECTION);
    }

    /**
     * Sets the deflector to a target angle in degrees.
     * @param angle target angle in degrees
     */
    public void setAngleInDegrees(double angle) {
        this.targetPosition = MathUtils.clamp(
                angle,
                0.0,
                1.0
        );
        this.state = DeflectorState.MOVING;
    }

    /**
     * Sets the deflector to a target angle in radians.
     * @param angle target angle in radians
     */
    public void setAngleInRadians(double angle) {
        setAngleInDegrees(Math.toDegrees(angle));
    }

    /**
     * Directly sets a custom servo position, overriding angle-based control.
     * @param position value between 0.0 and 1.0
     */
    public void setPosition(double position) {
        this.targetPosition = MathUtils.clamp(position, 0.0, 1.0);
        this.state = DeflectorState.CUSTOM;
    }

    /** Returns the deflector to its idle position. */
    public void idle() {
        this.targetPosition = SubsystemsConfig.Deflector.IDLE_POSITION;
        this.state = DeflectorState.IDLE;
    }

    /** Returns the current state of the deflector. */
    public DeflectorState getState() { return this.state; }

    /** Returns the current staged target position. */
    public double getTargetPosition() { return this.targetPosition; }

    /** Applies the staged position to hardware. Must be called every loop. */
    @Override
    public void update() {
        this.deflector.setPosition(this.targetPosition);
    }
}