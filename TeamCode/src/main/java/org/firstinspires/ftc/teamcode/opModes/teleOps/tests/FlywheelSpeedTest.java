package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name = "Flywheel Speed Test", group = "Flywheel")
public class FlywheelSpeedTest extends OpMode {

    private Flywheel flywheel;
    private double targetRPM = 0.0;

    private static final double STEP_COARSE = 100.0;
    private static final double STEP_FINE   = 10.0;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())    targetRPM += STEP_COARSE;
        if (gamepad1.dpadDownWasPressed())  targetRPM -= STEP_COARSE;
        if (gamepad1.dpadRightWasPressed()) targetRPM += STEP_FINE;
        if (gamepad1.dpadLeftWasPressed())  targetRPM -= STEP_FINE;

        targetRPM = Math.max(0.0, targetRPM);

        if (gamepad1.aWasPressed()) flywheel.setRPM(targetRPM);
        if (gamepad1.bWasPressed()) flywheel.stop();

        flywheel.update();

        telemetry.addLine("Flywheel Speed Test");
        telemetry.addData("target RPM",  "%.1f", targetRPM);
        telemetry.addData("current RPM", "%.1f", flywheel.getRPM());
        telemetry.addData("state",       flywheel.getState());
        telemetry.addData("at speed",    flywheel.isAtSpeed());
        telemetry.addLine("DPAD UP/DOWN = ±100 RPM | DPAD RIGHT/LEFT = ±10 RPM | A = go | B = stop");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}