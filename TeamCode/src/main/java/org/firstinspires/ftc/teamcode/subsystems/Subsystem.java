package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Base contract for all robot subsystems.
 * Every subsystem must implement {@link #update()} to apply
 * staged state to hardware. This method must be called every loop.
 */
public interface Subsystem {
    void update();
}