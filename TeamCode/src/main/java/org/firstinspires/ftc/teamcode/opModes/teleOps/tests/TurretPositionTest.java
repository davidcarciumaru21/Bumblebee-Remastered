package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Test OpMode for turret positioning.
 * Use DPAD UP/DOWN to increment/decrement target angle.
 * Use Y to return to idle (center position).
 */
@TeleOp(name = "Turret Position Test", group = "Turret")
public class TurretPositionTest extends OpMode {

    private Turret turret;
    private double targetAngle = 0.0;

    private static final double STEP = 5.0;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())   targetAngle += STEP;
        if (gamepad1.dpadDownWasPressed()) targetAngle -= STEP;
        if (gamepad1.yWasPressed())        turret.idle();

        targetAngle = Math.max(
                SubsystemsConfig.Turret.MIN_ANGLE,
                Math.min(SubsystemsConfig.Turret.MAX_ANGLE, targetAngle)
        );

        turret.setTargetAngle(targetAngle);
        turret.update();

        telemetry.addLine("Turret Position Test");
        telemetry.addData("current angle", turret.getAngle());
        telemetry.addData("target angle",   "%.1f°", targetAngle);
        telemetry.addData("target position","%.4f",  turret.getTargetPosition());
        telemetry.addData("state",          turret.getState());
        telemetry.addLine("DPAD UP/DOWN = ±5° | Y = idle");
        telemetry.update();
    }
}