package org.firstinspires.ftc.teamcode.opModes.teleOps.production;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Main TeleOp — uses Robot.java for all subsystem control.
 *
 * Controls:
 * Gamepad1:
 * - Left stick      — drive
 * - Right stick X   — rotate
 * - Share           — toggle field centric / robot centric
 * - Right trigger   — intake (pull game elements)
 * - Left trigger    — eject (push game elements out)
 * - Right bumper    — shoot
 * - Left bumper     — stop shooting
 */
@TeleOp(name = "Main TeleOp", group = "production")
public class MainTeleOp extends OpMode {

    private Robot   robot;
    private boolean fieldCentric = true;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry.addLine("Robot initialized — waiting for start");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.setPosition();
        telemetry.addLine("Pose loaded");
        telemetry.addData("alliance", robot.getAllianceColor());
        telemetry.addData("goal pose x", robot.getGoalPose().getX());
        telemetry.addData("goal pose y", robot.getGoalPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {

        // toggle field centric / robot centric
        if (gamepad1.shareWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        // drive
        robot.move(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                fieldCentric
        );

        // intake
        if (!robot.isShooting()) {

            if (gamepad1.right_trigger > 0.1) {
                robot.feed();

            } else if (gamepad1.left_trigger > 0.1) {
                robot.eject();

            } else {
                robot.stopFeed();
            }
        }

        // shooting
        if (gamepad1.rightBumperWasPressed())  robot.shoot();
        if (gamepad1.leftBumperWasPressed())   robot.stopShooting();

        // update
        robot.update();

        // telemetry
        telemetry.addLine("=== Main TeleOp ===");
        telemetry.addData("mode",       fieldCentric ? "field centric" : "robot centric");
        telemetry.addData("alliance",   robot.getAllianceColor());
        telemetry.addData("shooting",   robot.isShooting());
        telemetry.addData("pose x",     "%.2f", robot.getFollower().getPose().getX());
        telemetry.addData("pose y",     "%.2f", robot.getFollower().getPose().getY());
        telemetry.addData("pose h",     "%.2f°", Math.toDegrees(robot.getFollower().getPose().getHeading()));
        telemetry.addData("distance",   "%.2f in", robot.getFollower().getPose().distanceFrom(robot.getGoalPose()));
        telemetry.update();
    }
}