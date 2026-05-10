package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;

/**
 * Controls the voltage sensor with an exponential filter.
 * Filtered voltage is updated once per loop in {@link #update()}.
 * All subsystems requiring voltage compensation should use this instead of reading directly.
 */
public class VoltageSensor implements Subsystem {

    private final com.qualcomm.robotcore.hardware.VoltageSensor voltageSensor;
    private double filteredVoltage = SubsystemsConfig.VoltageSensor.INITIAL_FILTERED_VOLTAGE;

    public VoltageSensor(HardwareMap hardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /** Returns the current filtered voltage. */
    public double getVoltage() { return this.filteredVoltage; }

    /** Updates the exponential filter. Must be called every loop. */
    @Override
    public void update() {
        filteredVoltage += SubsystemsConfig.VoltageSensor.VOLTAGE_ALPHA
                * (voltageSensor.getVoltage() - filteredVoltage);
    }
}