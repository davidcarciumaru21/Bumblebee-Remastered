package org.firstinspires.ftc.teamcode.managers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.IntakingManagerState;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Manages the intake and indexer subsystems.
 * Handles pulling, shooting pull, reversing, and automatic stall detection.
 *
 * <p>Stall detection is active only in PULL state.
 * When stall is detected (3 balls assumed), transitions to IDLE automatically.
 * Can be reactivated immediately by calling pull() again.
 *
 * State machine:
 * - IDLE        — intake and indexer at idle power
 * - PULL        — intake at full pull, indexer at idle, stall detection active
 * - SHOOT_PULL  — intake and indexer at full pull, no stall detection
 * - REVERSE     — intake and indexer at push power
 */
public class IntakingManager {

    private final Intake      intake;
    private final Indexer     indexer;
    private final ElapsedTime stallTimer = new ElapsedTime();

    private IntakingManagerState state = IntakingManagerState.IDLE;

    public IntakingManager(Intake intake, Indexer indexer) {
        this.intake  = intake;
        this.indexer = indexer;
    }

    /** Starts pulling game elements. Stall detection is active in this state. */
    public void pull() {
        this.state = IntakingManagerState.PULL;
        this.stallTimer.reset();
    }

    /** Activates full pull for shooting. Stall detection is disabled. */
    public void shootPull() {
        this.state = IntakingManagerState.SHOOT_PULL;
    }

    /** Reverses intake and indexer to eject game elements. */
    public void reverse() {
        this.state = IntakingManagerState.REVERSE;
    }

    /** Stops and returns to idle power. */
    public void idle() {
        this.state = IntakingManagerState.IDLE;
    }

    /** Returns the current state of the intaking manager. */
    public IntakingManagerState getState() { return this.state; }

    /** Returns true if the intaking manager is idle. */
    public boolean isIdle() { return this.state == IntakingManagerState.IDLE; }

    /** Must be called every loop. */
    public void update() {
        switch (state) {
            case IDLE:
                intake.idle();
                indexer.idle();
                break;

            case PULL:
                if (intake.getCurrent() >= SubsystemsConfig.Intake.STALL_CURRENT_AMPS
                        && stallTimer.milliseconds() > SubsystemsConfig.Intake.STALL_TIME_MS) {
                    intake.idle();
                    indexer.idle();
                    state = IntakingManagerState.IDLE;
                    break;
                }

                intake.pull();
                indexer.idle();
                break;

            case SHOOT_PULL:
                intake.pull();
                indexer.pull();
                break;

            case REVERSE:
                intake.push();
                indexer.push();
                break;
        }

        intake.update();
        indexer.update();
    }
}