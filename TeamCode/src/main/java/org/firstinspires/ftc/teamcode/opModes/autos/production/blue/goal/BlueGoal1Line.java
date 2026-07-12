package org.firstinspires.ftc.teamcode.opModes.autos.production.blue.goal;

import com.google.gson.JsonObject;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfig;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.managersEnums.ShootingManagerState;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.utils.AutoTimeouts.*;

@Autonomous(name = "Blue Goal 1 Line", group = "Blue")
public class BlueGoal1Line extends OpMode {

    private static final Pose START_POSE = new Pose(31.176, 132.122, Math.toRadians(270.0));

    private Follower follower;
    private AutoPaths paths;

    private final Pose closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
    private final Pose farGoalPose      = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;

    private VoltageSensor voltageSensor;
    private Flywheel      flywheel;
    private Deflector     deflector;
    private Turret        turret;
    private Stopper       stopper;
    private Intake        intake;
    private Indexer       indexer;

    private ShootingManager shootingManager;
    private IntakingManager intakingManager;

    public static class AutoPaths {
        public final PathChain startToPreloadShoot;
        public final PathChain preloadShootToFirstLineApproach;
        public final PathChain firstLineIntake;
        public final PathChain firstLineToShoot;

        public AutoPaths(Follower follower) {
            startToPreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(31.176, 132.122),
                            new Pose(42.538, 98.080)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(137.0))
                    .build();

            preloadShootToFirstLineApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(42.538, 98.080),
                            new Pose(48.030, 83.032),
                            new Pose(40.093, 82.500)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(137.0), Math.toRadians(180.0))
                    .build();

            firstLineIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(40.093, 82.500),
                            new Pose(21.000, 82.500)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                    .build();

            firstLineToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21.000, 82.500),
                            new Pose(55.400, 82.500)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(137.0))
                    .build();
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();

        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
        deflector     = new Deflector(hardwareMap);
        turret        = new Turret(hardwareMap);
        stopper       = new Stopper(hardwareMap);
        intake        = new Intake(hardwareMap);
        indexer       = new Indexer(hardwareMap);

        intakingManager = new IntakingManager(intake, indexer);
        shootingManager = new ShootingManager(flywheel, deflector, turret, stopper, intakingManager);

        paths = new AutoPaths(follower);
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
    }

    @Override
    public void start() {
        schedule(
                sequential(
                        followWithTimeout(follower, paths.startToPreloadShoot, DEFAULT_PATH_TIMEOUT_MS),
                        shootThreeBalls(),

                        followWithTimeout(follower, paths.preloadShootToFirstLineApproach, DEFAULT_PATH_TIMEOUT_MS),
                        parallel(
                                instant(() -> intakingManager.pull()),
                                followWithTimeout(follower, paths.firstLineIntake, false, 0.5, LINE_INTAKE_TIMEOUT_MS)
                        ),
                        instant(() -> intakingManager.idle()),

                        followWithTimeout(follower, paths.firstLineToShoot, DEFAULT_PATH_TIMEOUT_MS),
                        shootThreeBalls()
                )
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();

        voltageSensor.update();
        follower.update();
        updateShootingTarget();
    }

    @Override
    public void stop() {
        writeEndPose("BLUE");
        Scheduler.reset();
    }

    private Command shootThreeBalls() {
        return sequential(
                waitUntil(() -> turret.isAtPosition()),
                instant(() -> shootingManager.shoot()),
                waitUntil(() -> shootingManager.isShooting()),
                waitMs(SubsystemsConfig.Flywheel.THREE_BALL_SHOT_TIME_MS),
                instant(() -> shootingManager.stop()),
                waitUntil(() -> shootingManager.getState() == ShootingManagerState.IDLE)
        );
    }

    private void updateShootingTarget() {
        Pose currentPose = follower.getPose();
        Pose goalPose = currentPose.distanceFrom(closeMidGoalPose) < ShootingConfig.Mid.MAX_DISTANCE
                ? closeMidGoalPose
                : farGoalPose;

        double distance = currentPose.distanceFrom(goalPose)
                + SubsystemsConfig.RobotDimensions.TurreToCenterDistance;
        double globalAngleToGoal = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );

        double angleToGoal = globalAngleToGoal - currentPose.getHeading();
        angleToGoal = Math.atan2(Math.sin(angleToGoal), Math.cos(angleToGoal));

        Vector velocityVector = follower.poseTracker.getVelocity().copy();
        velocityVector.rotateVector(-currentPose.getHeading());
        shootingManager.update(distance, velocityVector, angleToGoal);
    }

    private void writeEndPose(String color) {
        Pose endPose = follower.getPose();

        JsonObject json = new JsonObject();
        json.addProperty("x", endPose.getX());
        json.addProperty("y", endPose.getY());
        json.addProperty("heading", endPose.getHeading());
        json.addProperty("color", color);

        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(json.toString());
        } catch (IOException ignored) {}
    }
}
