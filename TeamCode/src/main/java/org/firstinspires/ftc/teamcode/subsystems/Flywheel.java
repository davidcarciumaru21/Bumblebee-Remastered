package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.FlywheelState;

/**
 * Controls the flywheel motors responsible for launching game elements.
 *
 * <p>Control algorithm — PF with voltage compensation and RPM ramping:
 *
 * 1. RPM RAMPING:
 *    Instead of jumping directly to the target RPM, the controller ramps up
 *    gradually by limiting the maximum RPM change per second (MAX_ACCEL_RPM_PER_SEC).
 *    This prevents current spikes and mechanical stress on the motors.
 *    rampedRPM approaches targetRPM at a controlled rate each loop.
 *
 * 2. FEEDFORWARD (ff):
 *    Predicts the power needed to reach the target RPM without waiting for error to build up.
 *    Two components:
 *    - KS * signum(rampedRPM): static feedforward — overcomes friction regardless of speed.
 *    - KV * rampedRPM:         velocity feedforward — scales linearly with target speed.
 *    Tune KS first (minimum power to move), then KV (power per RPM at steady state).
 *
 * 3. PROPORTIONAL CORRECTION (p):
 *    Corrects the remaining error between rampedRPM and currentRPM.
 *    p = KP * (rampedRPM - currentRPM)
 *    Tune KP last — too high causes oscillation, too low causes steady-state error.
 *
 * 4. VOLTAGE COMPENSATION:
 *    Battery voltage drops under load, which would cause the flywheel to slow down.
 *    An exponential filter smooths the voltage reading to avoid noise:
 *    filteredVoltage += VOLTAGE_ALPHA * (measuredVoltage - filteredVoltage)
 *    The total power output is divided by filteredVoltage to normalize it,
 *    so the same RPM target produces consistent behavior regardless of battery level.
 *
 * 5. AT_SPEED detection:
 *    Once the error between targetRPM and currentRPM is within AT_SPEED_TOLERANCE,
 *    the state transitions to AT_SPEED — used by ShootingManager to know when to feed.
 *
 * State machine:
 * - IDLE         — flywheel holds at idle power (configurable, can be non-zero)
 * - RAMPING_UP   — flywheel is accelerating toward target RPM
 * - AT_SPEED     — flywheel has reached target RPM within tolerance
 * - CUSTOM_POWER — flywheel runs at a fixed raw power, bypassing PF control
 */
public class Flywheel implements Subsystem {

    private final DcMotorEx     motor1;
    private final DcMotorEx     motor2;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime   timer = new ElapsedTime();

    private FlywheelState state = FlywheelState.IDLE;

    private double targetRPM       = 0.0;
    private double rampedRPM       = 0.0;
    private double filteredVoltage = SubsystemsConfig.VoltageSensor.INITIAL_FILTERED_VOLTAGE;
    private double customPower     = 0.0;
    private double lastTime        = 0.0;

    // tunable live — initialized from config, can be overridden by tuners
    private double kP = SubsystemsConfig.Flywheel.KP;

    public Flywheel(HardwareMap hardwareMap) {
        this.motor1 = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Flywheel.MOTOR_NAME_1);
        this.motor2 = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Flywheel.MOTOR_NAME_2);

        this.motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.lastTime = timer.seconds();
    }

    /**
     * Sets the target RPM directly.
     * @param rpm target RPM
     */
    public void setRPM(double rpm) {
        this.targetRPM = rpm;
        this.state = FlywheelState.RAMPING_UP;
    }

    /**
     * Sets the target RPM based on ball speed in inches per second.
     * @param speed ball speed in inches per second
     */
    public void setSpeedInchesPerSecond(double speed) {
        this.targetRPM = speed;
        this.state = FlywheelState.RAMPING_UP;
    }

    /**
     * Directly sets motor power, bypassing PF control.
     * @param power value between 0.0 and 1.0
     */
    public void setPower(double power) {
        this.customPower = power;
        this.state = FlywheelState.CUSTOM_POWER;
    }

    /** Stops the flywheel and resets all control state. */
    public void stop() {
        this.targetRPM = 0.0;
        this.rampedRPM = 0.0;
        this.state     = FlywheelState.IDLE;
        setRawPower(SubsystemsConfig.Flywheel.IDLE_POWER);
    }

    /**
     * Sets KP live. Use only for tuning.
     * @param kP proportional gain
     */
    public void setKP(double kP) { this.kP = kP; }

    /** Returns the current state of the flywheel. */
    public FlywheelState getState() { return this.state; }

    /** Returns true if the flywheel has reached its target RPM within the configured tolerance. */
    public boolean isAtSpeed() { return this.state == FlywheelState.AT_SPEED; }

    /** Returns the average RPM across both motors. */
    public double getRPM() {
        return ((motor1.getVelocity() + motor2.getVelocity()) / 2.0 * 60.0)
                / SubsystemsConfig.Flywheel.TICKS_PER_REV;
    }

    /** Returns the current target RPM. */
    public double getTargetRPM() { return this.targetRPM; }

    /** Applies the staged control to hardware. Must be called every loop. */
    @Override
    public void update() {
        double currentTime = timer.seconds();
        double deltaTime   = currentTime - lastTime;
        lastTime           = currentTime;

        switch (state) {
            case IDLE:
                setRawPower(SubsystemsConfig.Flywheel.IDLE_POWER);
                break;

            case RAMPING_UP:
            case AT_SPEED:
                double currentRPM = getRPM();

                // ramp toward target RPM at a limited rate to avoid current spikes
                double maxStep = SubsystemsConfig.Flywheel.MAX_ACCEL_RPM_PER_SEC * deltaTime;
                double diff    = targetRPM - rampedRPM;
                rampedRPM     += Range.clip(diff, -maxStep, maxStep);

                // feedforward: predict power needed based on target speed
                double ff    = SubsystemsConfig.Flywheel.KS * Math.signum(rampedRPM)
                        + SubsystemsConfig.Flywheel.KV * rampedRPM;

                // proportional correction: fix remaining error between ramped and actual RPM
                double error = rampedRPM - currentRPM;
                double p     = this.kP * error;

                // exponential voltage filter: normalize output to compensate for battery drop
                filteredVoltage += SubsystemsConfig.VoltageSensor.VOLTAGE_ALPHA
                        * (voltageSensor.getVoltage() - filteredVoltage);

                double power = Range.clip((ff + p) / filteredVoltage, 0.0, 1.0);
                setRawPower(power);

                // transition to AT_SPEED once error is within tolerance
                if (Math.abs(targetRPM - currentRPM) <= SubsystemsConfig.Flywheel.AT_SPEED_TOLERANCE) {
                    state = FlywheelState.AT_SPEED;
                } else {
                    state = FlywheelState.RAMPING_UP;
                }
                break;

            case CUSTOM_POWER:
                setRawPower(this.customPower);
                break;
        }
    }

    private void setRawPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }
}