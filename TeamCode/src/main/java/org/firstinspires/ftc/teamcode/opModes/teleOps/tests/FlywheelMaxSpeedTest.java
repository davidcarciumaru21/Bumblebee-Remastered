package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name = "Flywheel Max Speed Test", group = "Flywheel")
public class FlywheelMaxSpeedTest extends OpMode {

    private Flywheel flywheel;

    private static final double TARGET_RPM = 6000;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void start() {
        flywheel.setRPM(TARGET_RPM);
    }

    @Override
    public void loop() {
        flywheel.update();

        telemetry.addLine("Flywheel Max Speed Test");
        telemetry.addData("target RPM",  TARGET_RPM);
        telemetry.addData("current RPM", "%.1f", flywheel.getRPM());
        telemetry.addData("state",       flywheel.getState());
        telemetry.addData("at speed",    flywheel.isAtSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}