package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfigurations;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.AllianceColor;
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
import java.io.FileReader;
import java.io.IOException;

/**
 * Central robot class. Initializes all subsystems and managers.
 * Exposes high-level methods for use in OpModes.
 *
 * Usage:
 * 1. Create Robot in init()
 * 2. Call setPosition() in start() to load pose from JSON
 * 3. Call update() every loop
 */
public class Robot {

    // subsystems
    private final VoltageSensor voltageSensor;
    private final Intake        intake;
    private final Indexer       indexer;
    private final Stopper       stopper;
    private final Deflector     deflector;
    private final Flywheel      flywheel;
    private final Turret        turret;

    // managers
    private final IntakingManager intakingManager;
    private final ShootingManager shootingManager;

    // pedro pathing
    private final Follower follower;

    // alliance and goal pose
    private AllianceColor allianceColor = AllianceColor.RED;
    private Pose          goalPose      = ShootingConfigurations.Goals.RED_GOAL_POSE;

    public Robot(HardwareMap hardwareMap) {
        // subsystems
        voltageSensor = new VoltageSensor(hardwareMap);
        intake        = new Intake(hardwareMap);
        indexer       = new Indexer(hardwareMap);
        stopper       = new Stopper(hardwareMap);
        deflector     = new Deflector(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
        turret        = new Turret(hardwareMap, voltageSensor);

        // follower
        follower = Constants.createFollower(hardwareMap);

        // managers
        intakingManager = new IntakingManager(intake, indexer);
        shootingManager = new ShootingManager(flywheel, deflector, turret, stopper, intakingManager);
    }

    /**
     * Reads starting pose and alliance color from JSON saved at end of autonomous.
     * Falls back to default pose and RED alliance if file is missing or corrupt.
     * Call this in start().
     */
    public void setPosition() {
        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileReader reader = new FileReader(file)) {
            JsonObject json = new JsonParser().parse(reader).getAsJsonObject();

            double x       = json.get("x").getAsDouble();
            double y       = json.get("y").getAsDouble();
            double heading = json.get("heading").getAsDouble();
            String color   = json.get("color").getAsString();

            follower.setStartingPose(new Pose(x, y, heading));

            if (color.equalsIgnoreCase("BLUE")) {
                allianceColor = AllianceColor.BLUE;
                goalPose      = ShootingConfigurations.Goals.BLUE_GOAL_POSE;
            } else {
                allianceColor = AllianceColor.RED;
                goalPose      = ShootingConfigurations.Goals.RED_GOAL_POSE;
            }

        } catch (IOException e) {
            // fallback — default pose and RED alliance
            follower.setStartingPose(new Pose(0, 0, 0));
            allianceColor = AllianceColor.BLUE;
            goalPose      = ShootingConfigurations.Goals.BLUE_GOAL_POSE;
        }

        follower.startTeleOpDrive();
    }

    // --- shooting ---

    /** Starts shooting sequence. */
    public void shoot() {
        shootingManager.shoot();
    }

    /** Stops shooting. */
    public void stopShooting() {
        shootingManager.stop();
    }

    /** Returns true if currently shooting. */
    public boolean isShooting() {
        return shootingManager.isShooting();
    }

    // --- intake ---

    /** Starts pulling game elements. */
    public void feed() {
        intakingManager.pull();
    }

    /** Ejects game elements. */
    public void eject() {
        intakingManager.reverse();
    }

    /** Stops intake. */
    public void stopFeed() {
        intakingManager.idle();
    }

    // --- drive ---

    /**
     * Sets drivetrain movement vectors.
     * @param forward      forward/backward power
     * @param strafe       left/right power
     * @param rotation     rotational power
     * @param fieldCentric true = field centric, false = robot centric
     */
    public void move(double forward, double strafe, double rotation, boolean fieldCentric) {
        follower.setTeleOpDrive(forward, strafe, rotation, fieldCentric);
    }

    // --- getters ---

    public AllianceColor getAllianceColor() { return allianceColor; }
    public Follower      getFollower()      { return follower; }
    public Pose          getGoalPose()      { return goalPose; }

    /**
     * Updates all subsystems and managers.
     * Must be called every loop.
     */
    public void update() {
        // voltage first — all subsystems depend on it
        voltageSensor.update();

        // follower
        follower.update();

        // calculate distance and angle to goal
        Pose   currentPose  = follower.getPose();
        double distance     = currentPose.distanceFrom(goalPose);
        double angleToGoal  = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );
        Vector velocityVector = follower.poseTracker.getVelocity();

        // update managers
        shootingManager.update(distance, velocityVector, angleToGoal);
    }
}