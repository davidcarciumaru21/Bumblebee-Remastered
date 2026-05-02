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
 * Uses a PF controller with voltage compensation and RPM ramping.
 * State is staged internally and applied to hardware only in {@link #update()}.
 */
public class Flywheel implements Subsystem {

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();

    private FlywheelState state = FlywheelState.IDLE;

    private double targetRPM       = 0.0;
    private double rampedRPM       = 0.0;
    private double filteredVoltage = SubsystemsConfig.VoltageSensor.INITIAL_FILTERED_VOLTAGE;
    private double customPower     = 0.0;
    private double lastTime        = 0.0;

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
        this.targetRPM = 10.446 * speed + 198.20486; // TODO: replace with real values from calculations
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

                double maxStep = SubsystemsConfig.Flywheel.MAX_ACCEL_RPM_PER_SEC * deltaTime;
                double diff    = targetRPM - rampedRPM;
                rampedRPM     += Range.clip(diff, -maxStep, maxStep);

                double ff    = SubsystemsConfig.Flywheel.KS * Math.signum(rampedRPM)
                        + SubsystemsConfig.Flywheel.KV * rampedRPM;
                double error = rampedRPM - currentRPM;
                double p     = SubsystemsConfig.Flywheel.KP * error;

                filteredVoltage += SubsystemsConfig.VoltageSensor.VOLTAGE_ALPHA
                        * (voltageSensor.getVoltage() - filteredVoltage);

                double power = Range.clip((ff + p) / filteredVoltage, 0.0, 1.0);
                setRawPower(power);

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