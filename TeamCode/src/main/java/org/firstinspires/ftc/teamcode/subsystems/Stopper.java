package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.subsystemsEnums.StopperState;

/**
 * Controls the stopper servo responsible for blocking or releasing game elements.
 * Position is staged internally and applied to hardware only in {@link #update()}.
 * State is determined by elapsed time after a command is issued.
 */
public class Stopper implements Subsystem {

    private final Servo stopper;
    private final ElapsedTime timer = new ElapsedTime();

    private StopperState state = StopperState.CLOSED;
    private double customPosition = 0.0;

    public Stopper(HardwareMap hardwareMap) {
        this.stopper = hardwareMap.get(Servo.class, SubsystemsConfig.Stopper.SERVO_NAME);
    }

    /** Commands the stopper to open. State updates to OPEN after configured time. */
    public void open() {
        this.state = StopperState.OPENING;
        this.timer.reset();
    }

    /** Commands the stopper to close. State updates to CLOSED after configured time. */
    public void close() {
        this.state = StopperState.CLOSING;
        this.timer.reset();
    }

    /**
     * Directly sets a custom servo position, overriding open/close state.
     * @param position value between 0.0 and 1.0
     */
    public void setPosition(double position) {
        this.customPosition = position;
        this.state = StopperState.CUSTOM;
    }

    /** Returns the current state of the stopper. */
    public StopperState getState() { return this.state; }

    /** Returns the current servo position. */
    public double getPosition() { return this.stopper.getPosition(); }

    /** Returns true if the stopper has fully opened. */
    public boolean isOpen() { return this.state == StopperState.OPEN; }

    /** Returns true if the stopper has fully closed. */
    public boolean isClosed() { return this.state == StopperState.CLOSED; }

    /** Applies the staged position to hardware. Must be called every loop. */
    @Override
    public void update() {
        switch (state) {
            case OPENING:
                this.stopper.setPosition(SubsystemsConfig.Stopper.OPEN_POSITION);
                if (timer.milliseconds() >= SubsystemsConfig.Stopper.TIME_TO_OPEN_MS) {
                    this.state = StopperState.OPEN;
                }
                break;
            case CLOSING:
                this.stopper.setPosition(SubsystemsConfig.Stopper.CLOSED_POSITION);
                if (timer.milliseconds() >= SubsystemsConfig.Stopper.TIME_TO_CLOSE_MS) {
                    this.state = StopperState.CLOSED;
                }
                break;
            case OPEN:
                this.stopper.setPosition(SubsystemsConfig.Stopper.OPEN_POSITION);
                break;
            case CLOSED:
                this.stopper.setPosition(SubsystemsConfig.Stopper.CLOSED_POSITION);
                break;
            case CUSTOM:
                this.stopper.setPosition(this.customPosition);
                break;
        }
    }
}