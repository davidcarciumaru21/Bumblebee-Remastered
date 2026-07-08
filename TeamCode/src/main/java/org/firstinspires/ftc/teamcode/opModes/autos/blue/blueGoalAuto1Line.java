package org.firstinspires.ftc.teamcode.opModes.autos.blue;

import com.google.gson.JsonObject;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfig;
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
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Blue autonomous routine based on the project template, using the Pedro Pathing
 * Visualizer coordinates for the blue goal one-line route.
 */
@Autonomous(name = "blueGoalAuto1Line", group = "Blue")
public class blueGoalAuto1Line extends OpMode {
    private static final Pose START_POSE = new Pose(31.176489096573214, 132.12227414330215, Math.toRadians(270));

    private Follower follower;
    private AutoPaths paths;

    private final Pose closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
    private final Pose farGoalPose = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;

    private Flywheel flywheel;
    private Deflector deflector;
    private Turret turret;
    private Stopper stopper;
    private Intake intake;
    private Indexer indexer;
    private VoltageSensor voltageSensor;

    private ShootingManager shootingManager;
    private IntakingManager intakingManager;

    public static class AutoPaths {
        public final PathChain startToPreloadShoot;
        public final PathChain shootToLineIntake;

        public AutoPaths(Follower follower) {
            startToPreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(31.176489096573214, 132.12227414330215),
                            new Pose(42.53816199376948, 98.08021806853584)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(137))
                    .build();

            shootToLineIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(42.53816199376948, 98.08021806853584),
                            new Pose(48.029935123835074, 83.03205815641665),
                            new Pose(39.87295435981967, 82.49791693588621)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .addPath(new BezierLine(
                            new Pose(39.87295435981967, 82.49791693588621),
                            new Pose(18.073208722741434, 82.21105919003115)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(new BezierLine(
                            new Pose(18.073208722741434, 82.21105919003115),
                            new Pose(52.82477214290333, 82.67860266647627)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();

        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        deflector = new Deflector(hardwareMap);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);

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
        telemetry.addLine("Started auto");
        telemetry.update();

        schedule(
                sequential(
                        follow(follower, paths.startToPreloadShoot),
                        instant(() -> shootingManager.shoot()),
                        waitMs(600),
                        parallel(
                                instant(() -> intakingManager.pull()),
                                follow(follower, paths.shootToLineIntake)
                        ),
                        instant(() -> intakingManager.idle())
                )
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();
        voltageSensor.update();
        follower.update();

        Pose currentPose = follower.getPose();
        Pose goalPose = currentPose.distanceFrom(closeMidGoalPose) < ShootingConfig.Mid.MAX_DISTANCE
                ? closeMidGoalPose
                : farGoalPose;

        double distance = currentPose.distanceFrom(goalPose);
        double globalAngleToGoal = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );
        double angleToGoal = globalAngleToGoal - currentPose.getHeading();
        angleToGoal = Math.atan2(Math.sin(angleToGoal), Math.cos(angleToGoal));
        Vector velocityVector = follower.poseTracker.getVelocity();

        shootingManager.update(distance, velocityVector, angleToGoal);
        intakingManager.update();

        telemetry.addData("Auto", "blueGoalAuto1Line");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        Pose endPose = follower.getPose();

        JsonObject json = new JsonObject();
        json.addProperty("x", endPose.getX());
        json.addProperty("y", endPose.getY());
        json.addProperty("heading", endPose.getHeading());
        json.addProperty("color", "BLUE");

        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(json.toString());
        } catch (IOException ignored) {
        }

        Scheduler.reset();
    }
}
