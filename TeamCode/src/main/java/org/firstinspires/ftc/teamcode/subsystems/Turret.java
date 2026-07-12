package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.subsystemsEnums.TurretState;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * Controls the tracking turret using only external encoder feedback.
 * A proportional controller moves the positional-servo command based on angular error.
 */
public class Turret implements Subsystem {

    private final Servo turret;
    private final DcMotor encoder;
    private final ElapsedTime timer = new ElapsedTime();

    private TurretState state           = TurretState.IDLE;
    private double      targetAngle     = 0.0;
    private double      angleError      = 0.0;
    private double      targetPosition  = SubsystemsConfig.Turret.IDLE_POSITION;
    private double      lastUpdateTime  = 0.0;

    public Turret(HardwareMap hardwareMap) {
        this.encoder = hardwareMap.get(DcMotor.class, SubsystemsConfig.Turret.ENCODER_NAME);
        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.turret = hardwareMap.get(Servo.class, SubsystemsConfig.Turret.SERVO_NAME);
        this.turret.setPosition(SubsystemsConfig.Turret.IDLE_POSITION);
        this.lastUpdateTime = timer.seconds();
    }

    /**
     * Sets the target alignment angle. The encoder controller applies it in {@link #update()}.
     * @param angle target angular displacement in degrees relative to the chassis forward vector
     */
    public double getAngle() {
        return encoder.getCurrentPosition() / SubsystemsConfig.Turret.ENCODER_TICKS_PER_DEGREE;
    }

    public void setTargetAngle(double angle) {
        setTargetAngle(angle, 0.0);
    }

    public void setTargetAngle(double angle, double outOfBoundsFallbackAngle) {
        double selectedAngle = angle;
        if (angle < SubsystemsConfig.Turret.MIN_ANGLE || angle > SubsystemsConfig.Turret.MAX_ANGLE) {
            selectedAngle = outOfBoundsFallbackAngle;
        }

        this.targetAngle = MathUtils.clamp(
                selectedAngle,
                SubsystemsConfig.Turret.MIN_ANGLE,
                SubsystemsConfig.Turret.MAX_ANGLE
        );
        this.state = TurretState.AT_POSITION;
    }

    /** Returns the structural tracking array back to its hardware center home index (0.5). */
    public void idle() {
        this.targetAngle = 0.0;
        this.state       = TurretState.IDLE;
    }

    /** Returns true when encoder feedback is inside the configured angular tolerance. */
    public boolean isAtPosition() {
        return Math.abs(this.angleError) <= SubsystemsConfig.Turret.ANGLE_TOLERANCE_DEGREES;
    }

    /** Fetches the immediate functional state machine identifier of this structural subsystem. */
    public TurretState getState() { return this.state; }

    /** Exposes the currently computed target hardware position ready to be pushed downstream. */
    public double getTargetPosition() { return this.targetPosition; }

    public double getAnglePosition() { return this.targetAngle; }

    /** Returns the latest target-minus-current encoder error in degrees. */
    public double getAngleError() { return this.angleError; }

    /**
     * Moves the servo command incrementally using only proportional encoder error.
     */
    @Override
    public void update() {
        double currentTime = timer.seconds();
        double deltaTime = MathUtils.clamp(currentTime - lastUpdateTime, 0.001, 0.05);
        lastUpdateTime = currentTime;

        double currentAngle = getAngle();
        this.angleError = this.targetAngle - currentAngle;

        double servoPositionRate = SubsystemsConfig.Turret.ENCODER_KP * this.angleError;

        servoPositionRate = MathUtils.clamp(
                servoPositionRate,
                -SubsystemsConfig.Turret.MAX_SERVO_POSITION_RATE,
                SubsystemsConfig.Turret.MAX_SERVO_POSITION_RATE
        );

        if (isAtPosition()) {
            servoPositionRate = 0.0;
        }

        this.targetPosition = MathUtils.clamp(
                this.targetPosition + servoPositionRate * deltaTime,
                0.0,
                1.0
        );

        this.turret.setPosition(this.targetPosition);
    }
}
