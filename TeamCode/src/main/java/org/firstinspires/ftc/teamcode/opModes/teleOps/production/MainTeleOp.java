package org.firstinspires.ftc.teamcode.opModes.teleOps.production;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;

/**
 * Main TeleOp operations controller. Handles dual gamepad controller mapping structures,
 * scales input power variables dynamically based on driver modifier inputs,
 * and maintains continuous updates to tracking registers.
 */
@TeleOp(name = "Main TeleOp", group = "production")
public class MainTeleOp extends OpMode {

    private Robot   robot;
    private boolean fieldCentric = true;
    private boolean previousGamepad2RightStickButton = false;
    private String  limelightResetStatus = "not requested";

    @Override
    public void init() {
        // Instantiate the main control coordination wrapper
        robot = new Robot(hardwareMap);
        telemetry.addLine("Robot initialized — waiting for start");
        telemetry.update();
    }

    @Override
    public void start() {
        // Retrieve persistent autonomous tracking parameters and apply to the localizer
        robot.setPosition();

        telemetry.addLine("Pose loaded");
        telemetry.addData("alliance", robot.getAllianceColor());
        telemetry.addData("goal pose x", robot.getGoalPose().getX());
        telemetry.addData("goal pose y", robot.getGoalPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {

        // =========================================================================
        // GAMEPAD 1: PRIMARY CHASSIS TRANSLATION AND ELEMENT INTAKE CONTROLS
        // =========================================================================

        // Share button toggles coordinate mapping between field-relative and robot-relative modes
        if (gamepad1.shareWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        // Precision Slow Mode Trigger Evaluation
        // If left_trigger passes raw analog deadzone bounds, clamp input powers to the configuration scalar
        double speedScalar = 1.0;
        if (gamepad1.left_trigger > 0.1) {
            speedScalar = SubsystemsConfig.Drivetrain.SLOW_MODE_COEFFICIENT;
        }

        // Apply stick controls to drivetrain and scale by the calculated modifier
        robot.driver.move(
                -gamepad1.left_stick_y * speedScalar,
                -gamepad1.left_stick_x * speedScalar,
                -gamepad1.right_stick_x * speedScalar,
                fieldCentric
        );

        // Core Intake and Collection Processing Block
        // Evaluates actions strictly when the shooting manager state machine is idle to protect components
        if (!robot.driver.isShooting()) {
            if (gamepad1.right_trigger > 0.1) {
                // Collect elements inward
                robot.driver.feed();
            } else if (gamepad1.left_bumper) {
                // Eject elements outward via the left bumper modifier
                robot.driver.eject();
            } else {
                // Maintain passive idle holding force
                robot.driver.stopFeed();
            }
        }

        // Active Automated Firing Activation sequence
        if (gamepad1.rightBumperWasPressed()) {
            robot.driver.shoot();
        }

        // Context-Based Firing Abort Override
        // If left_bumper is triggered while automated firing sequences are running, forcefully terminate shooting actions
        if (robot.driver.isShooting() && gamepad1.leftBumperWasPressed()) {
            robot.driver.stopShooting();
        }


        // =========================================================================
        // GAMEPAD 2: AUXILIARY SUBSYSTEM DIAGNOSTICS AND FIELD RELOCALIZATION
        // =========================================================================

        // Establish binary evaluation registers for the multi-stage hardware safety matrix
        boolean safetyMatrixEnabled = gamepad2.left_trigger > 0.5;
        boolean resetMatrixEnabled  = gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5;
        boolean limelightResetPressed = gamepad2.right_stick_button && !previousGamepad2RightStickButton;

        // Subsystem Diagnostic Execution Envelope: Active only when the safety trigger criteria is met
        if (safetyMatrixEnabled) {

            // --- CRITICAL ZONE: COLD-START FIELD RELOCALIZATION VECTOR INTERLOCK ---
            // Absolute odometry overrides are structurally isolated; requires both triggers depressed to bypass locks
            if (resetMatrixEnabled) {
                if (gamepad2.xWasPressed()) {
                    robot.emergency.resetX();        // Recalibrate X axis absolute coordinate flush against perimeter wall
                }
                if (gamepad2.yWasPressed()) {
                    robot.emergency.resetY();        // Recalibrate Y axis absolute coordinate flush against perimeter wall
                }
                if (gamepad2.bWasPressed()) {
                    robot.emergency.resetHeading();  // Recalibrate absolute orientation/theta parameter via coplanar reference
                }
                if (limelightResetPressed) {
                    limelightResetStatus = robot.emergency.resetPoseFromLimelight() ? "applied" : "no valid pose";
                }
            }

            // --- STANDARD DIAGNOSTIC OPERATIONS ZONE ---
            // Accessible via standard safety matrix authorization (Left Trigger held independently)

            // Manual Turret Alignment Fine Tuning Adjustment Arrays via D-Pad Horizontal Axes
            if (gamepad2.dpadLeftWasPressed())   robot.emergency.changeTurretOffset(-1.0);
            if (gamepad2.dpadRightWasPressed())  robot.emergency.changeTurretOffset(1.0);

            // Reset calibration registers or apply structural homing overrides to the tracking turret assembly
            if (gamepad2.dpadDownWasPressed())   robot.emergency.resetTurretOffset();
            if (gamepad2.dpadUpWasPressed())     robot.emergency.forceTurretToZero();

            // Isolated Internal Indexing Roller Mechanical Manual Overrides
            if (gamepad2.rightBumperWasPressed()) robot.emergency.forceIndexerStart();
            if (gamepad2.leftBumperWasPressed())  robot.emergency.forceIndexerStop();

            // Isolated Hardware Servo Component Adjustments (Re-mapped to prevent trigger cross-talk conflicts)
            if (gamepad2.aWasPressed())          robot.emergency.forceStopperOpen();
            if (gamepad2.backWasPressed())       robot.emergency.forceStopperClose();
        }


        // =========================================================================
        // SYSTEM SYNC AND COMPONENT POLLING STATUS TELEMETRY OUTPUTS
        // =========================================================================

        // Process internal update math registers across the structural framework
        robot.update();

        // Render system telemetry metrics out onto the operator terminal display screen
        telemetry.addLine("=== Main TeleOp ===");
        telemetry.addData("mode",       fieldCentric ? "field centric" : "robot centric");
        telemetry.addData("alliance",   robot.getAllianceColor());
        telemetry.addData("shooting",   robot.driver.isShooting());
        telemetry.addData("pose x",     "%.2f", robot.getFollower().getPose().getX());
        telemetry.addData("pose y",     "%.2f", robot.getFollower().getPose().getY());
        telemetry.addData("pose h",     "%.2f°", Math.toDegrees(robot.getFollower().getPose().getHeading()));
        telemetry.addData("distance",   "%.2f in", robot.getFollower().getPose().distanceFrom(robot.getGoalPose()));
        telemetry.addData("turret offset", "%.1f°", robot.emergency.getTurretOffset());
        telemetry.addData("limelight target", robot.getLimelight().hasTarget());
        telemetry.addData("limelight yaw", "%.2f", robot.getLimelight().getYaw());
        telemetry.addData("limelight reset", limelightResetStatus);
        telemetry.addData("safety lock", safetyMatrixEnabled ? (resetMatrixEnabled ? "UNLOCKED (RESET ENABLED)" : "DIAGNOSTICS ONLY") : "LOCKED");
        telemetry.update();

        previousGamepad2RightStickButton = gamepad2.right_stick_button;
    }
}
