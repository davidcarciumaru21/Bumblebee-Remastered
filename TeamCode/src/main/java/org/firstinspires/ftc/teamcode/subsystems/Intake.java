package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.IntakeState;

/**
 * Controls the intake motor responsible for collecting and ejecting game elements.
 * Power is staged internally and applied to hardware only in {@link #update()}.
 */
public class Intake implements Subsystem {

    private final DcMotor intake;
    private IntakeState state = IntakeState.IDLE;
    private double customPower = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.intake = hardwareMap.get(DcMotor.class, SubsystemsConfig.Intake.MOTOR_NAME);
        this.intake.setDirection(SubsystemsConfig.Intake.DIRECTION);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Runs the intake inward to collect game elements. */
    public void pull() { this.state = IntakeState.PULLING; }

    /** Runs the intake outward to eject game elements. */
    public void push() { this.state = IntakeState.PUSHING; }

    /** Stops the intake motor. */
    public void idle() { this.state = IntakeState.IDLE; }

    /**
     * Directly sets a custom motor power, overriding pull/push/idle state.
     * @param power value between -1.0 and 1.0
     */
    public void setPower(double power) {
        this.customPower = power;
        this.state = IntakeState.CUSTOM;
    }

    /** Returns the current state of the intake. */
    public IntakeState getState() { return this.state; }

    /** Applies the staged power to hardware. Must be called every loop. */
    @Override
    public void update() {
        switch (state) {
            case PULLING:
                this.intake.setPower(SubsystemsConfig.Intake.PULL_POWER);
                break;
            case PUSHING:
                this.intake.setPower(SubsystemsConfig.Intake.PUSH_POWER);
                break;
            case IDLE:
                this.intake.setPower(SubsystemsConfig.Intake.IDLE_POWER);
                break;
            case CUSTOM:
                this.intake.setPower(this.customPower);
                break;
        }
    }
}