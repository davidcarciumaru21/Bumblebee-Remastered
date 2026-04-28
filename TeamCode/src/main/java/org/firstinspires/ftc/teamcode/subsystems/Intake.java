package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;

/**
 * Controls the intake motor responsible for collecting and ejecting game elements.
 * Power is staged internally and applied to hardware only in {@link #update()}.
 */
public class Intake implements Subsystem {

    private final DcMotor intake;
    private double power = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.intake = hardwareMap.get(DcMotor.class, SubsystemsConfig.Intake.MOTOR_NAME);
        this.intake.setDirection(SubsystemsConfig.Intake.DIRECTION);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Runs the intake inward to collect game elements. */
    public void pull() { this.power = SubsystemsConfig.Intake.PULL_POWER; }

    /** Runs the intake outward to eject game elements. */
    public void push() { this.power = SubsystemsConfig.Intake.PUSH_POWER; }

    /** Stops the intake motor. */
    public void idle() { this.power = SubsystemsConfig.Intake.IDLE_POWER; }

    /**
     * Directly sets motor power. Use only when pull/push/idle are insufficient.
     * @param power value between -1.0 and 1.0
     */
    public void setPower(double power) { this.power = power; }

    /** Applies the staged power to hardware. Must be called every loop. */
    @Override
    public void update() {
        this.intake.setPower(this.power);
    }
}