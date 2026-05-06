package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.subsystemsEnums.IndexerState;

/**
 * Controls the indexer motor responsible for feeding game elements into the flywheel.
 * Power is staged internally and applied to hardware only in {@link #update()}.
 */
public class Indexer implements Subsystem {

    private final DcMotor indexer;
    private IndexerState state = IndexerState.IDLE;
    private double customPower = 0.0;

    public Indexer(HardwareMap hardwareMap) {
        this.indexer = hardwareMap.get(DcMotor.class, SubsystemsConfig.Indexer.MOTOR_NAME);
        this.indexer.setDirection(SubsystemsConfig.Indexer.DIRECTION);
        this.indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Runs the indexer inward to feed game elements toward the flywheel. */
    public void pull() { this.state = IndexerState.PULLING; }

    /** Runs the indexer outward to eject game elements. */
    public void push() { this.state = IndexerState.PUSHING; }

    /** Stops the indexer motor. */
    public void idle() { this.state = IndexerState.IDLE; }

    /**
     * Directly sets a custom motor power, overriding pull/push/idle state.
     * @param power value between -1.0 and 1.0
     */
    public void setPower(double power) {
        this.customPower = power;
        this.state = IndexerState.CUSTOM;
    }

    /** Returns the current state of the indexer. */
    public IndexerState getState() { return this.state; }

    /** Applies the staged power to hardware. Must be called every loop. */
    @Override
    public void update() {
        switch (state) {
            case PULLING:
                this.indexer.setPower(SubsystemsConfig.Indexer.PULL_POWER);
                break;
            case PUSHING:
                this.indexer.setPower(SubsystemsConfig.Indexer.PUSH_POWER);
                break;
            case IDLE:
                this.indexer.setPower(SubsystemsConfig.Indexer.IDLE_POWER);
                break;
            case CUSTOM:
                this.indexer.setPower(this.customPower);
                break;
        }
    }
}