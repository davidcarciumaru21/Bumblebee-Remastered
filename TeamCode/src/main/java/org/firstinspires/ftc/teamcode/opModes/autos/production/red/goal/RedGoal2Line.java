package org.firstinspires.ftc.teamcode.opModes.autos.production.red.goal;

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
import org.firstinspires.ftc.teamcode.utils.AutoUtils;

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

@Autonomous(name = "Red Goal 2 Line", group = "Red")
public class RedGoal2Line extends OpMode {

    private static final Pose START_POSE = mirroredPose(31.176, 132.122, 270.0);

    private Follower follower;
    private AutoPaths paths;

    private final Pose closeMidGoalPose = ShootingConfig.Goals.RED_GOAL_POSE;
    private final Pose farGoalPose      = ShootingConfig.Goals.RED_FAR_GOAL_POSE;

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
        public final PathChain firstShootToSecondLineApproach;
        public final PathChain secondLineIntake;
        public final PathChain secondLineToShoot;

        public AutoPaths(Follower follower) {
            startToPreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(31.176, 132.122),
                            mirroredPoint(42.538, 98.080)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(270.0), mirroredHeading(137.0))
                    .build();

            preloadShootToFirstLineApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            mirroredPoint(42.538, 98.080),
                            mirroredPoint(48.030, 83.032),
                            mirroredPoint(40.093, 82.500)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(137.0), mirroredHeading(180.0))
                    .build();

            firstLineIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(40.093, 82.500),
                            mirroredPoint(19.000, 82.500)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(180.0), mirroredHeading(180.0))
                    .build();

            firstLineToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(19.000, 82.500),
                            mirroredPoint(55.400, 82.500)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(180.0), mirroredHeading(137.0))
                    .build();

            firstShootToSecondLineApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            mirroredPoint(55.400, 82.500),
                            mirroredPoint(55.542, 57.526),
                            mirroredPoint(40.093, 58.550)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(137.0), mirroredHeading(180.0))
                    .build();

            secondLineIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(40.093, 58.550),
                            mirroredPoint(14, 54.362138284021114)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(180.0), mirroredHeading(180.0))
                    .build();

            secondLineToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            mirroredPoint(14, 54.362138284021114),
                            mirroredPoint(55.264, 57.301),
                            mirroredPoint(55.400, 82.500)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(180.0), mirroredHeading(137.0))
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
                        follow(follower, paths.startToPreloadShoot),
                        shootThreeBalls(),

                        follow(follower, paths.preloadShootToFirstLineApproach),
                        collectLine(paths.firstLineIntake),
                        follow(follower, paths.firstLineToShoot),
                        shootThreeBalls(),

                        follow(follower, paths.firstShootToSecondLineApproach),
                        collectLine(paths.secondLineIntake),
                        follow(follower, paths.secondLineToShoot),
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
        writeEndPose("RED");
        Scheduler.reset();
    }

    private Command collectLine(PathChain path) {
        return sequential(
                parallel(
                        instant(() -> intakingManager.pull()),
                        follow(follower, path, false, 0.5)
                ),
                instant(() -> intakingManager.idle())
        );
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

    private static Pose mirroredPoint(double blueX, double blueY) {
        Pose mirrored = new Pose(blueX, blueY).mirror();
        return new Pose(mirrored.getX(), mirrored.getY());
    }

    private static Pose mirroredPose(double blueX, double blueY, double blueHeadingDeg) {
        Pose mirrored = new Pose(blueX, blueY).mirror();
        return new Pose(mirrored.getX(), mirrored.getY(), mirroredHeading(blueHeadingDeg));
    }

    private static double mirroredHeading(double blueHeadingDeg) {
        return Math.toRadians(AutoUtils.mirrorHeading(blueHeadingDeg));
    }
}
