package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;

/**
 * Controls the indexer motor responsible for feeding game elements into the flywheel.
 * Power is staged internally and applied to hardware only in {@link #update()}.
 */
public class Indexer implements Subsystem {

    private final DcMotor indexer;
    private double power = 0.0;

    public Indexer(HardwareMap hardwareMap) {
        this.indexer = hardwareMap.get(DcMotor.class, SubsystemsConfig.Indexer.MOTOR_NAME);
        this.indexer.setDirection(SubsystemsConfig.Indexer.DIRECTION);
        this.indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Runs the indexer inward to feed game elements toward the flywheel. */
    public void pull() { this.power = SubsystemsConfig.Indexer.PULL_POWER; }

    /** Runs the indexer outward to eject game elements. */
    public void push() { this.power = SubsystemsConfig.Indexer.PUSH_POWER; }

    /** Stops the indexer motor. */
    public void idle() { this.power = SubsystemsConfig.Indexer.IDLE_POWER; }

    /**
     * Directly sets motor power. Use only when pull/push/idle are insufficient.
     * @param power value between -1.0 and 1.0
     */
    public void setPower(double power) { this.power = power; }

    /** Applies the staged power to hardware. Must be called every loop. */
    @Override
    public void update() {
        this.indexer.setPower(this.power);
    }
}
