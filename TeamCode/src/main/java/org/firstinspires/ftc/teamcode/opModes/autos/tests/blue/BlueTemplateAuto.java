package org.firstinspires.ftc.teamcode.opModes.autos.tests.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;

// Ivy Infrastructure Engine — Static Functional Pipeline Targets
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfig;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;

// Structural JSON Persistence Engines for Inter-OpMode State Handoff
import com.google.gson.JsonObject;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Deterministic Autonomous routine executing state-guarded command pipelines via the Ivy infrastructure engine.
 * Governs localized trajectory tracking, real-time kinematics/vector tracking, and cross-subsystem coordination
 * for the Blue Alliance perimeter configuration, culminating in an absolute state-serialization handoff.
 */
@Disabled
@Autonomous(name = "Blue Template Auto", group = "Blue")
public class BlueTemplateAuto extends OpMode {

    // Absolute Tracking & Pathing Infrastructure
    private Follower follower;
    private AutoPaths paths;

    // Range-Based Field Target Configuration for Blue Alliance Coordinates
    private final Pose closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
    private final Pose farGoalPose      = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;

    // Core Low-Level Hardware Subsystem Wrappers
    private Flywheel      flywheel;
    private Deflector     deflector;
    private Turret        turret;
    private Stopper       stopper;
    private Intake        intake;
    private Indexer       indexer;
    private VoltageSensor voltageSensor;

    // High-Level Subsystem Lifecycle Managers
    private ShootingManager shootingManager;
    private IntakingManager intakingManager;

    /**
     * Isolated trajectory factory. Generates parametric Bézier splines mapped
     * against absolute boundary thresholds configured for Blue Alliance coordinates.
     */
    public static class AutoPaths {
        public final PathChain startToPreloadShoot;
        public final PathChain shootToLineIntake;

        public AutoPaths(Follower follower) {
            // Path 1: Dispatches lateral translation vectors to advance the chassis to the optimal preload clearing zone
            startToPreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(17.951, 118.677),
                            new Pose(45.000, 112.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-37))
                    .build();

            // Path 2: Executes a coordinated rotational sweep to profile and ingest elements from the horizontal array
            shootToLineIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(45.000, 112.000),
                            new Pose(52.000, 95.000),
                            new Pose(16.500, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(180))
                    .build();
        }
    }

    /**
     * Instantiates the physical components, aggregates dependent pipelines,
     * and seeds initial localization tracking metrics.
     */
    @Override
    public void init() {
        // Enforce structural tracking initialization via static follower profiles
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(17.951, 118.677, Math.toRadians(-37)));
        follower.update();

        // Instantiate isolated physical subsystem control registers
        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
        deflector     = new Deflector(hardwareMap);
        turret        = new Turret(hardwareMap);
        stopper       = new Stopper(hardwareMap);
        intake        = new Intake(hardwareMap);
        indexer       = new Indexer(hardwareMap);

        // Bind dependencies into localized manager envelopes to coordinate cross-subsystem behaviors
        intakingManager = new IntakingManager(intake, indexer);
        shootingManager = new ShootingManager(flywheel, deflector, turret, stopper, intakingManager);

        // Pre-compute parametric curves prior to active loop transitions to safeguard thread constraints
        paths = new AutoPaths(follower);
    }

    /** Polled continuously during the init phase to maintain hardware registry readiness. */
    @Override
    public void init_loop() {
        Scheduler.execute();
    }

    /** Dispatches the non-blocking execution pipelines into the active scheduler bus. */
    @Override
    public void start() {
        telemetry.addLine("Started auto");
        telemetry.update();

        // Register the asynchronous command topology to run sequentially inside the scheduler core
        schedule(
                sequential(
                        // Step 1: Enforce closed-loop path tracking to the preload scoring vector
                        follow(follower, paths.startToPreloadShoot),

                        // Step 2: Fire intercept sequence via the manager interface and wait out physical tolerances
                        instant(() -> shootingManager.shoot()),
                        waitMs(600),

                        // Step 3 & 4: Interlock path tracking and hardware collection routines inside a parallel envelope
                        parallel(
                                instant(() -> intakingManager.pull()),
                                follow(follower, paths.shootToLineIntake)
                        ),

                        // Step 5: Terminate intake collection profiles upon absolute path arrival
                        instant(() -> intakingManager.idle())
                )
        );
    }

    /** Main loop execution interface. Monitored continuously by the FTC system kernel. */
    @Override
    public void loop() {
        // Evaluate the active command topology and execute the current pipeline frame
        Scheduler.execute();

        // Read voltage first to ensure tracking data feeds clean conversion models
        voltageSensor.update();

        // Process dead-wheel tracking metrics and compute active coordinate drift
        follower.update();

        // Fetch the instantaneous position tracking pose from the localizer module
        Pose currentPose = follower.getPose();

        // Select the independent Far target once the robot leaves the Close and Mid zones
        double closeMidDistance = currentPose.distanceFrom(closeMidGoalPose);
        Pose goalPose = closeMidDistance < ShootingConfig.Mid.MAX_DISTANCE
                ? closeMidGoalPose
                : farGoalPose;

        // Calculate absolute radial distance vectors to the selected high-goal setup
        double distance = currentPose.distanceFrom(goalPose);

        // Calculate absolute field-centric angular alignment toward the goal structure
        double globalAngleToGoal = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );

        // Normalize global vectors relative to the robot's real-time heading rotation
        double angleToGoal = globalAngleToGoal - currentPose.getHeading();

        // Wrap angular results inside standard trigonometric limits [-PI, PI]
        angleToGoal = Math.atan2(Math.sin(angleToGoal), Math.cos(angleToGoal));

        // Pull active linear translation velocities to calculate lead targeting adjustments
        Vector velocityVector = follower.poseTracker.getVelocity();

        // Dispatch targeting computations to the shooting management engine
        shootingManager.update(distance, velocityVector, angleToGoal);

        // Commit newly formatted operational profiles down to the physical intake hardware bus
        intakingManager.update();
    }

    /**
     * Serializes terminal coordinate vectors and alliance parameters to disk.
     * Prevents tracking drift and establishes continuous field-centric alignment for TeleOp.
     */
    @Override
    public void stop() {
        Pose endPose = follower.getPose();

        JsonObject json = new JsonObject();
        json.addProperty("x",       endPose.getX());
        json.addProperty("y",       endPose.getY());
        json.addProperty("heading", endPose.getHeading());
        json.addProperty("color",   "BLUE");

        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(json.toString());
        } catch (IOException ignored) {}

        // Flush active registries to safely reset the scheduler for subsequent match operations
        Scheduler.reset();
    }
}
