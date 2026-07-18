package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfig;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 * Orchestration hub for the robot. Handles instantiation, cross-subsystem messaging,
 * tracking math conversions, and exposes control routing layers directly to teleoperated executors.
 */
public class Robot {

    // Subsystem hardware wrapper declarations
    private final VoltageSensor voltageSensor;
    private final Intake        intake;
    private final Indexer       indexer;
    private final Stopper       stopper;
    private final Deflector     deflector;
    private final Flywheel      flywheel;
    private final Turret        turret;
    private final Limelight     limelight;

    // High-level subsystem state machine managers
    private final IntakingManager intakingManager;
    private final ShootingManager shootingManager;

    // Integrated Pedro Pathing tracking system wrapper
    private final Follower follower;

    // Dynamic field properties mapped based on selected setup configuration
    private AllianceColor allianceColor    = AllianceColor.BLUE;
    private Pose          closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
    private Pose          farGoalPose      = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;
    private Pose          goalPose         = closeMidGoalPose;

    // Control structural routing abstraction interfaces
    public final DriverControls    driver;
    public final EmergencyControls emergency;

    /**
     * Initializes all underlying components and maps the localized infrastructure.
     * @param hardwareMap Context instance reference passed from running OpModes.
     */
    public Robot(HardwareMap hardwareMap) {
        // Instantiate foundational physical components
        voltageSensor = new VoltageSensor(hardwareMap);
        intake        = new Intake(hardwareMap);
        indexer       = new Indexer(hardwareMap);
        stopper       = new Stopper(hardwareMap);
        deflector     = new Deflector(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
        turret        = new Turret(hardwareMap);
        limelight     = new Limelight(hardwareMap);

        // Build the Pedro Pathing follower module from static configuration bindings
        follower = Constants.createFollower(hardwareMap);

        // Setup the localized manager pipelines to resolve cross-subsystem actions
        intakingManager = new IntakingManager(intake, indexer);
        shootingManager = new ShootingManager(flywheel, deflector, turret, stopper, intakingManager);

        // Bind driver and emergency interaction layouts
        this.driver    = new DriverControls();
        this.emergency = new EmergencyControls();
    }

    /**
     * Extracts persistent localization vectors stored at the completion of Autonomous routines.
     * Automatically coordinates alliance targets and registers initial tracking positions.
     * Safe-guards with default fallbacks if telemetry files are unreadable or missing.
     */

    public double getCurrentTurretPosition() {
        return turret.getAngle();
    }

    public double getTargetTurretPosition() {
        return turret.getAnglePosition();
    }

    public double getTurretError() {
        return turret.getAngleError();
    }

    public boolean isTurretAtPosition() {
        return turret.isAtPosition();
    }

    public double getFlywheelRPM() {
        return flywheel.getRPM();
    }

    public double getFlywheelTargetRPM() {
        return flywheel.getTargetRPM();
    }

    public double getIndexerVelocityTicksPerSecond() {
        return indexer.getVelocityTicksPerSecond();
    }

    public double getIndexerPower() {
        return indexer.getPower();
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void setPosition() {
        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileReader reader = new FileReader(file)) {
            JsonObject json = new JsonParser().parse(reader).getAsJsonObject();

            double x       = json.get("x").getAsDouble();
            double y       = json.get("y").getAsDouble();
            double heading = json.get("heading").getAsDouble();
            String color   = json.get("color").getAsString();
            double turretPosition = json.get("turret").getAsDouble();


            turret.setAngleOffset(turretPosition);

            // Push the extracted vector positions out to the absolute tracker layout
            follower.setStartingPose(new Pose(x, y, heading));

            // Map alliance structures and establish corresponding high-goal targets
            if (color.equalsIgnoreCase("BLUE")) {
                allianceColor    = AllianceColor.BLUE;
                closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
                farGoalPose      = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;
            } else {
                allianceColor    = AllianceColor.RED;
                closeMidGoalPose = ShootingConfig.Goals.RED_GOAL_POSE;
                farGoalPose      = ShootingConfig.Goals.RED_FAR_GOAL_POSE;
            }

        } catch (IOException e) {
            // Safe fallback defaults: Zero coordinate tracking on the Blue Alliance map
            follower.setStartingPose(new Pose(0, 0, 0));
            allianceColor    = AllianceColor.BLUE;
            closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
            farGoalPose      = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;
            turret.setAngleOffset(0.0);
        }

        goalPose = closeMidGoalPose;

        // Initialize teleoperation execution controls inside the localizer module
        follower.startTeleOpDrive();
    }

    // --- Core System Getters ---
    public AllianceColor getAllianceColor()     { return allianceColor; }
    public Follower      getFollower()          { return follower; }
    public Pose          getGoalPose()          { return goalPose; }
    public Limelight     getLimelight()         { return limelight; }

    /**
     * Runs localized updates across subsystems, tracking routines, and target calculations.
     * Needs to be polled continuously inside the execution loops of all running OpModes.
     */
    public void update() {
        // Read voltage first to ensure tracking data feeds clean conversion models
        voltageSensor.update();

        // Cache the latest vision frame for optional driver-triggered relocalization
        limelight.update();

        // Process dead-wheel tracking metrics and compute active coordinate drift
        follower.update();

        // Fetch the instantaneous position tracking pose from the localizer module
        Pose currentPose = follower.getPose();

        // Select the independent Far target once the robot leaves the Close and Mid zones
        double closeMidDistance = currentPose.distanceFrom(closeMidGoalPose);
        goalPose = closeMidDistance < ShootingConfig.Mid.MAX_DISTANCE
                ? closeMidGoalPose
                : farGoalPose;

        // Calculate absolute radial distance vectors to the selected high-goal setup
        double distance = currentPose.distanceFrom(goalPose)
                + SubsystemsConfig.RobotDimensions.TurreToCenterDistance;

        // Calculate absolute field-centric angular alignment toward the goal structure
        double globalAngleToGoal = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );

        // Normalize global vectors relative to the robot's real-time heading rotation
        double angleToGoal = globalAngleToGoal - currentPose.getHeading();

        // Overlay manual fine-tuning offsets injected by the operator console
        angleToGoal += Math.toRadians(emergency.turretOffsetDegrees);

        // Wrap angular results inside standard trigonometric limits [-PI, PI]
        angleToGoal = Math.atan2(Math.sin(angleToGoal), Math.cos(angleToGoal));

        // Pull active linear translation velocities to calculate lead targeting adjustments
        Vector velocityVector = follower.poseTracker.getVelocity().copy();
        velocityVector.rotateVector(-currentPose.getHeading());

        // Dispatch targeting computations to the shooting management engine
        shootingManager.setTurretClampFallbackAngle(emergency.getTurretClampFallbackAngle());
        shootingManager.update(distance, velocityVector, angleToGoal);
    }

    // =========================================================================
    // PRIMARY DRIVER FUNCTIONAL LAYOUT
    // =========================================================================
    public class DriverControls {

        // --- Active Shooting Interfaces ---
        public void shoot()         { shootingManager.shoot(); }
        public void stopShooting()  { shootingManager.stop(); }
        public boolean isShooting() { return shootingManager.isActive(); }

        // --- Active Intake/Exclusion Interfaces ---
        public void feed()          { intakingManager.pull(); }
        public void eject()         { intakingManager.reverse(); }
        public void stopFeed()      { intakingManager.idle(); }

        // --- Standard Translation Drive Vectors ---
        public void move(double forward, double strafe, double rotation, boolean fieldCentric) {
            follower.setTeleOpDrive(forward, strafe, rotation, fieldCentric);
        }
    }

    // =========================================================================
    // EMERGENCY AND WALL RELOCALIZATION CONTROLS
    // =========================================================================
    public class EmergencyControls {
        private double turretOffsetDegrees = 0.0;

        // --- Turret Calibration Triggers ---
        public void changeTurretOffset(double deltaDegrees) { this.turretOffsetDegrees += deltaDegrees; }
        public void resetTurretOffset()                     { this.turretOffsetDegrees = 0.0; }
        public double getTurretOffset()                     { return this.turretOffsetDegrees; }
        public boolean isTurretForcedToZero()               { return shootingManager.isTurretForcedToZero(); }
        private double getTurretClampFallbackAngle()         { return -this.turretOffsetDegrees; }

        // --- Turret Overrides ---
        public void forceTurretToZero()                     { shootingManager.forceTurretToZero(); }

        // --- Pedro Pathing Dynamic Wall Relocalization (0 - 144 Absolute Map) ---

        /**
         * Resets the absolute tracking coordinate along the X-axis.
         * Calculates position dynamically assuming the chassis is compressed flat against the X=0 perimeter.
         * The center of the robot is mathematically placed at exactly half its total structural length.
         */
        public void resetX() {
            Pose current = follower.getPose();
            double targetX = SubsystemsConfig.RobotDimensions.LENGTH / 2.0;
            follower.setPose(new Pose(targetX, current.getY(), current.getHeading()));
        }

        /**
         * Resets the absolute tracking coordinate along the Y-axis.
         * Calculates position dynamically assuming the chassis is compressed flat against the Y=0 perimeter.
         * The center of the robot is mathematically placed at exactly half its total structural width.
         */
        public void resetY() {
            Pose current = follower.getPose();
            double targetY = SubsystemsConfig.RobotDimensions.WIDTH / 2.0;
            follower.setPose(new Pose(current.getX(), targetY, current.getHeading()));
        }

        /**
         * Forces a programmatic update to the tracking localizer's heading calculations.
         * Aligns orientation values cleanly when flush up against flat structural surfaces.
         */
        public void resetHeading() {
            Pose current = follower.getPose();
            double targetHeading = 0.0;
            follower.setPose(new Pose(current.getX(), current.getY(), targetHeading));
        }

        /**
         * Uses the latest valid Limelight botpose to relocalize Pedro Pathing.
         * @return true if the follower pose was updated.
         */
        public boolean resetPoseFromLimelight() {
            limelight.update();
            Pose limelightPose = limelight.getPose();
            if (limelightPose == null) {
                return false;
            }

            follower.setPose(limelightPose);
            return true;
        }

        // --- Low-Level Isolated Hardware Hardware Manual Overrides ---
        public void forceStopperOpen()  { shootingManager.forceStopperOpen(); }
        public void forceStopperClose() { shootingManager.forceStopperClose(); }
        public void forceIndexerStart() { intakingManager.forceIndexerStart(); }
        public void forceIndexerStop()  { intakingManager.forceIndexerStop(); }
    }
}
