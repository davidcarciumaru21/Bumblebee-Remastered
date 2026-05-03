package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.TurretState;

/**
 * Controls the turret CRServo using a quadratic speed profile (twoXSqrd).
 *
 * <p>Algorithm explanation:
 * Instead of a PD controller, the turret uses a quadratic interpolation approach:
 *
 * 1. The angular error (deltaAngle) is normalized by a brake distance constant,
 *    producing an interpolator value between -1 and 1.
 *
 * 2. This interpolator is fed into twoXSqrd(x, vMin):
 *    - Computes x² as the raw speed, preserving the sign of x.
 *    - If x² is below vMin, returns vMin to prevent stalling near the target.
 *    - If x² exceeds 1.0, clamps to 1.0 to prevent overflow.
 *    - Otherwise returns x² with the correct sign.
 *
 * 3. The result is a smooth speed curve: fast when far, slow when close,
 *    with a guaranteed minimum power to overcome friction.
 *
 * 4. When the error is within the dead zone threshold, power is set to 0.
 *
 * State machine:
 * - IDLE      — turret holds at idle power (configurable)
 * - GOING_TO  — turret is actively moving toward target angle
 * - AT_PLACE  — turret has reached target angle within dead zone
 */
public class Turret implements Subsystem {

    private final CRServo       turret;
    private final DcMotorEx     encoder;
    private final VoltageSensor voltageSensor;

    private TurretState state          = TurretState.IDLE;
    private double      targetAngle    = 0.0;
    private double      filteredVoltage = SubsystemsConfig.VoltageSensor.INITIAL_FILTERED_VOLTAGE;

    // tunable live — initialized from config, can be overridden by tuners
    private double brakeDistance = SubsystemsConfig.Turret.BRAKE_DISTANCE;
    private double deadZone      = SubsystemsConfig.Turret.DEAD_ZONE;

    public Turret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(CRServo.class, SubsystemsConfig.Turret.SERVO_NAME);
        this.turret.setDirection(CRServo.Direction.REVERSE);

        this.encoder = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Turret.ENCODER_NAME);
        this.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * Sets the target angle and transitions to GOING_TO state.
     * Angle is clamped to the configured min/max range.
     * @param angle target angle in degrees
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = Math.max(
                SubsystemsConfig.Turret.MIN_ANGLE,
                Math.min(SubsystemsConfig.Turret.MAX_ANGLE, angle)
        );
        this.state = TurretState.GOING_TO;
    }

    /** Stops the turret and transitions to IDLE state. */
    public void idle() { this.state = TurretState.IDLE; }

    /** Directly sets raw power to the CRServo. Use only for testing. */
    public void setRawPower(double power) { this.turret.setPower(power); }

    /**
     * Sets the brake distance live. Use only for tuning.
     * @param brakeDistance angular distance in degrees at which deceleration begins
     */
    public void setBrakeDistance(double brakeDistance) { this.brakeDistance = brakeDistance; }

    /**
     * Sets the dead zone live. Use only for tuning.
     * @param deadZone angular error in degrees below which turret is considered at target
     */
    public void setDeadZone(double deadZone) { this.deadZone = deadZone; }

    /** Returns true if the turret has reached its target angle. */
    public boolean isAtPlace() { return this.state == TurretState.AT_PLACE; }

    /** Returns the current state of the turret. */
    public TurretState getState() { return this.state; }

    /** Returns the current angle of the turret in degrees. */
    public double getCurrentAngle() {
        return (double) encoder.getCurrentPosition()
                / SubsystemsConfig.Turret.GEAR_RATIO
                / SubsystemsConfig.Turret.TICKS_PER_REV
                * 360.0;
    }

    /** Returns the current target angle in degrees. */
    public double getTargetAngle() { return this.targetAngle; }

    /** Returns the raw encoder ticks. Useful for verifying encoder direction. */
    public int getEncoderTicks() { return this.encoder.getCurrentPosition(); }

    /** Returns the current filtered voltage. */
    public double getFilteredVoltage() { return this.filteredVoltage; }

    /**
     * Quadratic speed profile function.
     * Produces a smooth speed curve: fast when far, slow when close.
     * Guarantees a minimum power (vMin) to prevent stalling near the target.
     *
     * @param x     normalized error, between -1.0 and 1.0
     * @param vMin  minimum output power to overcome friction
     * @return      signed speed between -1.0 and 1.0
     */
    private double twoXSqrd(double x, double vMin) {
        if (x == 0) return 0;

        double speed = x * x;
        double sign  = x / Math.abs(x);

        if (vMin > Math.abs(speed))   return vMin * sign;
        else if (Math.abs(speed) > 1) return sign;
        else                          return speed * sign;
    }

    /** Applies the staged control to hardware. Must be called every loop. */
    @Override
    public void update() {

        filteredVoltage += SubsystemsConfig.VoltageSensor.VOLTAGE_ALPHA
                * (voltageSensor.getVoltage() - filteredVoltage);

        // convert MIN_POWER_VOLTS to raw power based on current voltage
        double minPower = SubsystemsConfig.Turret.MIN_POWER_VOLTS / filteredVoltage;

        switch (state) {
            case IDLE:
                turret.setPower(SubsystemsConfig.Turret.IDLE_POWER);
                break;

            case GOING_TO:
                double deltaAngle   = targetAngle - getCurrentAngle();
                double interpolator = deltaAngle / this.brakeDistance;

                // clamp interpolator to [-1, 1]
                if (Math.abs(interpolator) > 1)
                    interpolator = interpolator / Math.abs(interpolator);

                double speed = twoXSqrd(interpolator, minPower);

                if (Math.abs(deltaAngle) < this.deadZone) {
                    turret.setPower(0.0);
                    state = TurretState.AT_PLACE;
                } else {
                    turret.setPower(speed);
                }
                break;

            case AT_PLACE:
                turret.setPower(0.0);
                break;
        }
    }
}