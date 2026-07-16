package org.firstinspires.ftc.teamcode.managers;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.IntakingManagerState;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Coordinated lifecycle manager for collection and internal processing mechanics.
 * Implements a state-guarded execution envelope designed to prevent continuous frame polls
 * from prematurely stripping manual operator-directed emergency intervention flags.
 */
public class IntakingManager {

    private final Intake  intake;
    private final Indexer indexer;

    private IntakingManagerState state = IntakingManagerState.IDLE;

    // Prioritized low-level hardware configuration override registers
    private boolean indexerForcedStart = false;
    private boolean indexerForcedStop  = false;

    public IntakingManager(Intake intake, Indexer indexer) {
        this.intake  = intake;
        this.indexer = indexer;
    }

    /**
     * Initiates active collection mechanics to ingest game elements from the field perimeter.
     * Guarded by a state variance evaluation step to safeguard localized override tracking arrays.
     */
    public void pull() {
        if (this.state != IntakingManagerState.PULL) {
            this.state = IntakingManagerState.PULL;
            clearEmergencyOverrides();
        }
    }

    /**
     * Commands maximum throughput velocity across internal tracking arrays to feed active firing states.
     * Bypasses localized telemetry constraints to enforce uninterrupted programmatic continuity.
     */
    public void shootPull() {
        if (this.state != IntakingManagerState.SHOOT_PULL) {
            this.state = IntakingManagerState.SHOOT_PULL;
            clearEmergencyOverrides();
        }
    }

    /**
     * Executes systemic inversion vectors across all collection roller assemblies.
     * Utilized to eject jammed components or clear intake channels via negative power coefficients.
     */
    public void reverse() {
        if (this.state != IntakingManagerState.REVERSE) {
            this.state = IntakingManagerState.REVERSE;
            clearEmergencyOverrides();
        }
    }

    /**
     * Transitions internal operating states back to passive holding modes.
     * Drops motor duty-cycle demands to configurations limits to minimize residual thermal buildup.
     */
    public void idle() {
        if (this.state != IntakingManagerState.IDLE) {
            this.state = IntakingManagerState.IDLE;
            clearEmergencyOverrides();
        }
    }

    /**
     * Fully stops both collection motors. Unlike IDLE, this applies zero power and is used while
     * the shooting system waits for the flywheel and stopper before feeding balls.
     */
    public void stop() {
        if (this.state != IntakingManagerState.STOPPED) {
            this.state = IntakingManagerState.STOPPED;
            clearEmergencyOverrides();
        }
    }

    // =========================================================================
    // EMERGENCY OVERRIDE INTERFACES (DIRECT BUS BYPASS ROUTING)
    // =========================================================================

    /** Forces the internal indexer loop to engage positive velocity vectors, bypassing automated managers. */
    public void forceIndexerStart() {
        this.indexerForcedStart = true;
        this.indexerForcedStop  = false;
    }

    /** Forces the internal indexer loop to cease mechanical operations, arresting power delivery models. */
    public void forceIndexerStop() {
        this.indexerForcedStart = false;
        this.indexerForcedStop  = true;
    }

    /** Re-establishes normal automated execution paradigms by resetting localized override registers. */
    private void clearEmergencyOverrides() {
        this.indexerForcedStart = false;
        this.indexerForcedStop  = false;
    }

    /** Fetches the structural lifecycle state configuration currently processed by this controller. */
    public IntakingManagerState getState() { return this.state; }

    /** Evaluates if the subsystem state architecture is currently bound inside an inactive IDLE profile. */
    public boolean isIdle() { return this.state == IntakingManagerState.IDLE; }

    /**
     * Executes systemic evaluations and processes state-machine transitions across the subsystem array.
     * Intercepts standard automation profiles with active override parameters directly prior to bus transmission.
     */
    public void update() {
        switch (state) {
            case IDLE:
                intake.idle();
                indexer.idle();
                break;

            case STOPPED:
                intake.setPower(0.0);
                indexer.setPower(0.0);
                break;

            case PULL:
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

        // =========================================================================
        // HARDWARE OVERRIDE INJECTION INTERCEPT LAYER
        // =========================================================================
        // Directly overrides downstream servo or motor registries if manual parameters are locked
        if (indexerForcedStart) {
            indexer.pull();
        } else if (indexerForcedStop) {
            indexer.idle();
        }

        // Commit newly formatted operational profiles to physical hardware wrappers
        intake.update();
        indexer.update();
    }
}
