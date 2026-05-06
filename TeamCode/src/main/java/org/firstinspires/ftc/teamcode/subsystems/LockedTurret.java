package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.subsystemsEnums.LockedTurretState;

/**
 * Debug-only locked turret subsystem.
 * Locks the turret servo to a fixed position for testing purposes.
 * State is staged internally and applied to hardware only in {@link #update()}.
 */
public class LockedTurret implements Subsystem {

    private final Servo turret;
    private LockedTurretState state = LockedTurretState.LOCKED;
    private double targetPosition = SubsystemsConfig.LockedTurret.LOCKED_POSITION;

    public LockedTurret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(Servo.class, SubsystemsConfig.LockedTurret.SERVO_NAME);
    }

    /** Locks the turret to the configured fixed position. */
    public void lock() {
        this.targetPosition = SubsystemsConfig.LockedTurret.LOCKED_POSITION;
        this.state = LockedTurretState.LOCKED;
    }

    /**
     * Sets a custom position, overriding the configured locked position.
     * @param position value between 0.0 and 1.0
     */
    public void setPosition(double position) {
        this.targetPosition = Math.max(0.0, Math.min(1.0, position));
        this.state = LockedTurretState.CUSTOM;
    }

    /** Returns the current state of the locked turret. */
    public LockedTurretState getState() { return this.state; }

    /** Returns the current staged target position. */
    public double getTargetPosition() { return this.targetPosition; }

    /** Applies the staged position to hardware. Must be called every loop. */
    @Override
    public void update() {
        switch (state) {
            case LOCKED:
                this.turret.setPosition(SubsystemsConfig.LockedTurret.LOCKED_POSITION);
                break;
            case CUSTOM:
                this.turret.setPosition(this.targetPosition);
                break;
        }
    }
}