package org.firstinspires.ftc.teamcode.opModes.autos.production.blue.base;

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

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.followWithTimeout;

@Autonomous(name = "Blue Base 1 Line Base Cycle 2", group = "Blue")
public class BlueBase1LineBaseCycle2 extends OpMode {

    private static final Pose START_POSE = new Pose(54.803, 8.661417, Math.toRadians(90.0));
    private static final double LINE_INTAKE_POWER = 0.5;
    private static final double BASE_INTAKE_POWER = 1.0;
    private static final double DEFAULT_PATH_TIMEOUT_MS = 5000.0;
    private static final double LINE_INTAKE_TIMEOUT_MS = 2500.0;
    private static final double BASE_INTAKE_TIMEOUT_MS = 5000.0;

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
        public final PathChain preloadShootToThirdLineApproach;
        public final PathChain thirdLineIntake;
        public final PathChain thirdLineToShoot;
        public final PathChain firstBaseIntake;
        public final PathChain firstBaseToShoot;
        public final PathChain secondBaseIntake;
        public final PathChain secondBaseToShoot;

        public AutoPaths(Follower follower) {
            preloadShootToThirdLineApproach = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.6, 8.9),
                            new Pose(54.6, 35.32398753894079)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(180.0))
                    .build();

            thirdLineIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.6, 35.32398753894079),
                            new Pose(14.0, 35.149)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                    .build();

            thirdLineToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(14.0, 35.149),
                            new Pose(52.395950155763245, 13.985669781931465)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(120.0))
                    .build();

            firstBaseIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(52.395950155763245, 13.985669781931465),
                            new Pose(44.91015169194866, 12.897316219369895),
                            new Pose(10.584230097901107, 8.511736565620435)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(120.0), Math.toRadians(180.0))
                    .build();

            firstBaseToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(8.584230097901107, 8.511736565620435),
                            new Pose(51.946432226092426, 13.714458892448398)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(120.0))
                    .build();

            secondBaseIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(51.946432226092426, 13.714458892448398),
                            new Pose(44.68311510293678, 28.306387310457577),
                            new Pose(.8649555060355185, 27.963659952504113)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(120.0), Math.toRadians(180.0))
                    .build();

            secondBaseToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(7.8649555060355185, 27.963659952504113),
                            new Pose(52.14488358513371, 13.636139089611241)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(120.0))
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
                        collectLine(paths.thirdLineIntake, LINE_INTAKE_POWER),
                        followWithTimeout(follower, paths.thirdLineToShoot, DEFAULT_PATH_TIMEOUT_MS),
                        shootThreeBalls(),

                        Groups.loop(
                                sequential(
                                        collectLine(paths.firstBaseIntake, BASE_INTAKE_POWER, BASE_INTAKE_TIMEOUT_MS),
                                        followWithTimeout(follower, paths.firstBaseToShoot, DEFAULT_PATH_TIMEOUT_MS),
                                        shootThreeBalls(),

                                        collectLine(paths.secondBaseIntake, BASE_INTAKE_POWER, BASE_INTAKE_TIMEOUT_MS),
                                        followWithTimeout(follower, paths.secondBaseToShoot, DEFAULT_PATH_TIMEOUT_MS),
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
        writeEndPose("BLUE");
        Scheduler.reset();
    }

    private Command collectLine(PathChain path, double maxPower) {
        return collectLine(path, maxPower, LINE_INTAKE_TIMEOUT_MS);
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
                waitUntil(() -> turret.isAtPosition() && flywheel.isAtSpeed()),
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
}
