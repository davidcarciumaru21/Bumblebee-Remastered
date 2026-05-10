package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp(name = "Flywheel Max Speed Test", group = "Flywheel")
public class FlywheelMaxSpeedTest extends OpMode {

    private Flywheel      flywheel;
    private VoltageSensor voltageSensor;

    private static final double TARGET_RPM = 6000;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
    }

    @Override
    public void start() {
        flywheel.setRPM(TARGET_RPM);
    }

    @Override
    public void loop() {
        voltageSensor.update();
        flywheel.update();

        telemetry.addLine("Flywheel Max Speed Test");
        telemetry.addData("target RPM",  TARGET_RPM);
        telemetry.addData("current RPM", "%.1f", flywheel.getRPM());
        telemetry.addData("state",       flywheel.getState());
        telemetry.addData("at speed",    flywheel.isAtSpeed());
        telemetry.addData("voltage",     "%.4f", voltageSensor.getVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}