package org.firstinspires.ftc.teamcode.managers;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfig;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.managersEnums.ShootingManagerState;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.Pair;

/**
 * Manages the shooting system: flywheel, deflector, turret, stopper, and intaking manager.
 *
 * <p>Flywheel, turret and deflector are updated constantly regardless of state.
 * Ballistic formula from Team 23435 Gyrobotic Droids PDF with velocity compensation.
 *
 * <p>Algorithm:
 * 1. Calculate initial launch angle α and velocity v0 from ballistic formula.
 * 2. Compensate for robot velocity — adjust vx and recalculate angle and speed.
 * 3. Apply deflector angle directly inside getTargetAngleAndVelocity.
 * 4. Return turret angle (angleToGoal + turretOffset) and launch speed.
 *
 * State machine:
 * - IDLE            — stopper closed, intaking manager idle
 * - ELEVATE_STOPPER — stopper opens, waits TIME_TO_OPEN_MS then transitions to SHOOTING
 * - SHOOTING        — intaking manager shoot pull, continues until stop() is called
 * - CLOSE_STOPPER   — stopper closes, intaking manager idle, waits TIME_TO_CLOSE_MS then IDLE
 */
public class ShootingManager {

    private final Flywheel        flywheel;
    private final Deflector       deflector;
    private final Turret          turret;
    private final Stopper         stopper;
    private final IntakingManager intakingManager;

    private ShootingManagerState state = ShootingManagerState.IDLE;
    private final ElapsedTime    timer = new ElapsedTime();

    // emergency flags
    private boolean turretForcedToZero = false;

    public ShootingManager(Flywheel flywheel,
                           Deflector deflector,
                           Turret turret,
                           Stopper stopper,
                           IntakingManager intakingManager) {
        this.flywheel        = flywheel;
        this.deflector       = deflector;
        this.turret          = turret;
        this.stopper         = stopper;
        this.intakingManager = intakingManager;
    }

    /** Starts shooting sequence. Transitions to ELEVATE_STOPPER. */
    public void shoot() {
        if (state == ShootingManagerState.IDLE) {
            state = ShootingManagerState.ELEVATE_STOPPER;
            stopper.open();
            timer.reset();
        }
    }

    /** Stops shooting. Transitions to CLOSE_STOPPER. */
    public void stop() {
        if (state == ShootingManagerState.SHOOTING) {
            state = ShootingManagerState.CLOSE_STOPPER;
            stopper.close();
            intakingManager.idle();
            timer.reset();
        }
    }

    // --- emergency overrides ---

    /** Toggles forcing the turret to point straight forward (0 degrees). */
    public void forceTurretToZero() {
        this.turretForcedToZero = !this.turretForcedToZero;
    }

    /** Forces state to IDLE and opens the stopper manually. */
    public void forceStopperOpen() {
        this.state = ShootingManagerState.IDLE;
        stopper.open();
    }

    /** Forces state to IDLE and closes the stopper manually. */
    public void forceStopperClose() {
        this.state = ShootingManagerState.IDLE;
        stopper.close();
    }

    /** Returns the current state of the shooting manager. */
    public ShootingManagerState getState() { return this.state; }

    /** Returns true if currently shooting. */
    public boolean isShooting() { return state == ShootingManagerState.SHOOTING; }

    /**
     * Calculates ballistic launch angle and velocity with velocity compensation.
     * Applies deflector angle directly.
     *
     * @param distance       distance to goal in inches
     * @param velocityVector robot velocity vector from Pedro Pathing
     * @param angleToGoal    angle from robot to goal in radians
     * @return Pair of (turret angle in radians, launch speed in inches/second)
     */
    private Pair<Double, Double> getTargetAngleAndVelocity(double distance,
                                                           Vector velocityVector,
                                                           double angleToGoal) {
        double height;
        double scoreAngle;
        double g = ShootingConfig.G;

        if (distance < ShootingConfig.Close.MAX_DISTANCE) {
            distance  -= ShootingConfig.Close.PASS_THROUGH_RADIUS;
            height     = ShootingConfig.Close.SCORE_HEIGHT;
            scoreAngle = ShootingConfig.Close.SCORE_ANGLE;
        } else if (distance < ShootingConfig.Mid.MAX_DISTANCE) {
            distance  -= ShootingConfig.Mid.PASS_THROUGH_RADIUS;
            height     = ShootingConfig.Mid.SCORE_HEIGHT;
            scoreAngle = ShootingConfig.Mid.SCORE_ANGLE;
        } else {
            distance  -= ShootingConfig.Far.PASS_THROUGH_RADIUS;
            height     = ShootingConfig.Far.SCORE_HEIGHT;
            scoreAngle = ShootingConfig.Far.SCORE_ANGLE;
        }

        // A. calculate initial launch angle and velocity
        double initialAngle = MathUtils.clamp(
                Math.atan(2 * height / distance - Math.tan(scoreAngle)),
                SubsystemsConfig.Deflector.MIN_ANGLE,
                SubsystemsConfig.Deflector.MAX_ANGLE
        );

        double initialV0 = Math.sqrt(
                (g * Math.pow(distance, 2)) /
                        (2 * Math.pow(Math.cos(initialAngle), 2) *
                                (distance * Math.tan(initialAngle) - height))
        );

        // B. velocity compensation
        double theta               = velocityVector.getTheta() - angleToGoal;
        double radialComponent     = -Math.cos(theta) * velocityVector.getMagnitude();
        double tangentialComponent =  Math.sin(theta) * velocityVector.getMagnitude();

        double vy   = initialV0 * Math.sin(initialAngle);
        double time = distance / (initialV0 * Math.cos(initialAngle));

        double vxCompensated = distance / time + radialComponent;
        double vxNew         = Math.sqrt(
                Math.pow(vxCompensated, 2) + Math.pow(tangentialComponent, 2)
        );

        double newDistance = vxNew * time;

        double newAngle = MathUtils.clamp(
                Math.atan(vy / vxNew),
                SubsystemsConfig.Deflector.MIN_ANGLE,
                SubsystemsConfig.Deflector.MAX_ANGLE
        );

        double newV0 = Math.sqrt(
                (g * Math.pow(newDistance, 2)) /
                        (2 * Math.pow(Math.cos(newAngle), 2) *
                                (newDistance * Math.tan(newAngle) - height))
        );

        double turretOffset = Math.atan2(tangentialComponent, vxCompensated);
        //double turretAngle  = Math.atan2(Math.sin(angleToGoal + turretOffset),
               // Math.cos(angleToGoal + turretOffset));
        double turretAngle  = 0;

        if (Double.isNaN(newAngle) || Double.isNaN(newV0)) {
            deflector.setAngleInRadians(SubsystemsConfig.Deflector.MIN_ANGLE);
            return new Pair<>(angleToGoal, 0.0);
        }

        deflector.setAngleInRadians(newAngle);
        return new Pair<>(turretAngle, newV0);
    }

    /**
     * Must be called every loop.
     * Flywheel, turret and deflector are updated constantly regardless of state.
     *
     * @param distance       distance to goal in inches
     * @param velocityVector robot velocity vector from Pedro Pathing
     * @param angleToGoal    angle from robot to goal in radians
     */
    public void update(double distance, Vector velocityVector, double angleToGoal) {

        // calculate and apply targets constantly regardless of state
        Pair<Double, Double> targets = getTargetAngleAndVelocity(distance, velocityVector, angleToGoal);

        // override turret targets if emergency force zero is active
        if (turretForcedToZero) {
            turret.setTargetAngle(0.0);
        } else {
            turret.setTargetAngle(-Math.toDegrees(targets.first));
        }

        flywheel.setSpeedInchesPerSecond(targets.second);

        // update subsystems
        flywheel.update();
        turret.update();
        deflector.update();
        stopper.update();
        intakingManager.update();

        // state machine
        switch (state) {
            case IDLE:
                intakingManager.idle();
                break;

            case ELEVATE_STOPPER:
                if (timer.milliseconds() >= SubsystemsConfig.Stopper.TIME_TO_OPEN_MS) {
                    state = ShootingManagerState.SHOOTING;
                }
                break;

            case SHOOTING:
                intakingManager.shootPull();
                break;

            case CLOSE_STOPPER:
                if (timer.milliseconds() >= SubsystemsConfig.Stopper.TIME_TO_CLOSE_MS) {
                    state = ShootingManagerState.IDLE;
                }
                break;
        }
    }
}