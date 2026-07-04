package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.subsystemsEnums.TurretState;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * Controls the tracking turret servo using a refined quadratic regression model.
 * Features an automated out-of-bounds boundary intercept that forces the assembly
 * back to mechanical zero whenever tracking vectors diverge past physical sweep limits.
 */
public class Turret implements Subsystem {

    private final Servo turret;
    private final DcMotor encoder;
    private double angle;

    private TurretState state          = TurretState.IDLE;
    private double      targetPosition = SubsystemsConfig.Turret.IDLE_POSITION;

    public Turret(HardwareMap hardwareMap) {
        this.encoder = hardwareMap.get(DcMotor.class, SubsystemsConfig.Turret.ENCODER_NAME);
        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.turret = hardwareMap.get(Servo.class, SubsystemsConfig.Turret.SERVO_NAME);
        this.turret.setPosition(SubsystemsConfig.Turret.IDLE_POSITION);
    }

    /**
     * Sets the target alignment angle and computes the corresponding servo duty cycle.
     * Evaluates boundary constraints; if the target falls outside permitted physical limits,
     * the system executes a fail-safe fallback routine, forcing the orientation to 0 degrees.
     * * @param angle Target angular displacement in degrees relative to the chassis forward vector.
     */

    public double getAngle(){
        double EncoderTicks = encoder.getCurrentPosition();
        double EncoderDegrees = EncoderTicks / 22.76 / 5.71;
        return EncoderDegrees;
    }

    public void setTargetAngle(double angle) {
        this.angle = angle;
        // Dynamic Boundary Evaluation: Check if the computed tracking vector is out of range
        if (angle < SubsystemsConfig.Turret.MIN_ANGLE || angle > SubsystemsConfig.Turret.MAX_ANGLE) {
            // Prevent mechanical straining or pinning at hardware hard-stops by returning home
            angle = 0.0;
        }


        this.targetPosition = MathUtils.clamp(
                0.0107467 * angle + 0.5,
                0.0,
                1.0
        );

        this.state = TurretState.AT_POSITION;
    }

    /** Returns the structural tracking array back to its hardware center home index (0.5). */
    public void idle() {
        this.targetPosition = SubsystemsConfig.Turret.IDLE_POSITION;
        this.state          = TurretState.IDLE;
    }

    /** Boolean flag indicating if the localized system is executing an active tracking state sequence. */
    public boolean isAtPosition() { return this.state == TurretState.AT_POSITION; }

    /** Fetches the immediate functional state machine identifier of this structural subsystem. */
    public TurretState getState() { return this.state; }

    /** Exposes the currently computed target hardware position ready to be pushed downstream. */
    public double getTargetPosition() { return this.targetPosition; }

    public double getAnglePosition() { return this.angle;}

    /** Forces the newly calculated position register onto the physical REV expansion hub bus layer. */
    @Override
    public void update() {
        this.turret.setPosition(this.targetPosition);
    }
}