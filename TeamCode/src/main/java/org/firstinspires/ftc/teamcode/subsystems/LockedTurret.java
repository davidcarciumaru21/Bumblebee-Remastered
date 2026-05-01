package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;

/**
 * Debug-only locked turret subsystem.
 * Locks the turret servo to a fixed position for testing purposes.
 * State is staged internally and applied to hardware only in {@link #update()}.
 */
public class LockedTurret implements Subsystem {

    private final Servo turret;
    private double targetPosition = SubsystemsConfig.LockedTurret.LOCKED_POSITION;

    public LockedTurret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(Servo.class, SubsystemsConfig.LockedTurret.SERVO_NAME);
    }

    /** Locks the turret to the configured fixed position. */
    public void lock() {
        this.targetPosition = SubsystemsConfig.LockedTurret.LOCKED_POSITION;
    }

    /** Applies the staged position to hardware. Must be called every loop. */
    @Override
    public void update() {
        this.turret.setPosition(this.targetPosition);
    }
}