package org.firstinspires.ftc.teamcode.opModes.autos.production.red.base;

import com.google.gson.JsonObject;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
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
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.followWithTimeout;

@Autonomous(name = "Red Base 1 Line Base Cycle", group = "Red")
public class RedBase1LineBaseCycle extends OpMode {

    private static final Pose START_POSE = mirroredPose(54.6, 8.9, 90.0);
    private static final double LINE_INTAKE_POWER = 0.5;
    private static final double BASE_INTAKE_POWER = 0.5;
    private static final double DEFAULT_PATH_TIMEOUT_MS = 5000.0;
    private static final double FLYWHEEL_READY_TIMEOUT_MS = 3000.0;
    private static final double LINE_INTAKE_TIMEOUT_MS = 2500.0;
    private static final double BASE_INTAKE_TIMEOUT_MS = 2500.0;
    private static final double CYCLE_PATH_TIMEOUT_MS = 5000.0;
    private static final double CYCLE_COLLECT_TIMEOUT_MS = 2500.0;

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
        public final PathChain preloadShootToThirdLineApproach;
        public final PathChain thirdLineIntake;
        public final PathChain thirdLineToShoot;
        public final PathChain shootToBaseApproach;
        public final PathChain baseIntake;
        public final PathChain baseToShoot;
        public final PathChain cycleShootToBaseApproach;
        public final PathChain cycleBaseIntake;
        public final PathChain cycleBaseToShoot;

        public AutoPaths(Follower follower) {
            preloadShootToThirdLineApproach = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(54.6, 8.9),
                            mirroredPoint(54.6, 34.82398753894079)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(90.0), mirroredHeading(180.0))
                    .build();

            thirdLineIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(54.6, 34.82398753894079),
                            mirroredPoint(14.0, 34.649)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(180.0), mirroredHeading(180.0))
                    .build();

            thirdLineToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(14.0, 34.649),
                            mirroredPoint(52.395950155763245, 11.985669781931465)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(180.0), mirroredHeading(120.0))
                    .build();

            shootToBaseApproach = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(52.395950155763245, 11.985669781931465),
                            mirroredPoint(9.186319325885822, 34.59085523515356)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(120.0), mirroredHeading(270.0))
                    .build();

            baseIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(9.186319325885822, 34.59085523515356),
                            mirroredPoint(8.084294403767442, 10.508986230125752)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(270.0), mirroredHeading(270.0))
                    .build();

            baseToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            mirroredPoint(8.084294403767442, 10.508986230125752),
                            mirroredPoint(27.59526246668123, 28.18580152627784),
                            mirroredPoint(52.395950155763245, 11.985669781931465)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(270.0), mirroredHeading(120.0))
                    .build();

            cycleShootToBaseApproach = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(52.395950155763245, 11.985669781931465),
                            mirroredPoint(9.186319325885822, 34.59085523515356)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(120.0), mirroredHeading(270.0))
                    .build();

            cycleBaseIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            mirroredPoint(9.186319325885822, 34.59085523515356),
                            mirroredPoint(8.084294403767442, 10.508986230125752)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(270.0), mirroredHeading(270.0))
                    .build();

            cycleBaseToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            mirroredPoint(8.084294403767442, 10.508986230125752),
                            mirroredPoint(27.59526246668123, 28.18580152627784),
                            mirroredPoint(52.395950155763245, 11.985669781931465)
                    ))
                    .setLinearHeadingInterpolation(mirroredHeading(270.0), mirroredHeading(120.0))
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
                        shootThreeBalls(),

                        followWithTimeout(follower, paths.preloadShootToThirdLineApproach, DEFAULT_PATH_TIMEOUT_MS),
                        collectLine(paths.thirdLineIntake, LINE_INTAKE_POWER, LINE_INTAKE_TIMEOUT_MS),
                        followWithTimeout(follower, paths.thirdLineToShoot, DEFAULT_PATH_TIMEOUT_MS),
                        shootThreeBalls(),

                        followWithTimeout(follower, paths.shootToBaseApproach, DEFAULT_PATH_TIMEOUT_MS),
                        collectLine(paths.baseIntake, BASE_INTAKE_POWER, BASE_INTAKE_TIMEOUT_MS),
                        followWithTimeout(follower, paths.baseToShoot, DEFAULT_PATH_TIMEOUT_MS),
                        shootThreeBalls(),

                        Groups.loop(
                                sequential(
                                        followWithTimeout(follower, paths.cycleShootToBaseApproach, CYCLE_PATH_TIMEOUT_MS),
                                        collectLine(paths.cycleBaseIntake, BASE_INTAKE_POWER, CYCLE_COLLECT_TIMEOUT_MS),
                                        followWithTimeout(follower, paths.cycleBaseToShoot, CYCLE_PATH_TIMEOUT_MS),
                                        shootThreeBalls()
                                )
                        )
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

    private Command collectLine(PathChain path, double maxPower, double timeoutMs) {
        return sequential(
                parallel(
                        instant(() -> intakingManager.pull()),
                        followWithTimeout(follower, path, false, maxPower, timeoutMs)
                ),
                instant(() -> intakingManager.idle())
        );
    }

    private Command shootThreeBalls() {
        return sequential(
                waitUntil(() -> turret.isAtPosition()),
                race(
                        waitUntil(() -> flywheel.isAtSpeed()),
                        waitMs(FLYWHEEL_READY_TIMEOUT_MS)
                ),
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
        json.addProperty("turret", turret.getAnglePosition());

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
