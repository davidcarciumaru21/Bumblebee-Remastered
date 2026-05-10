package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp(name = "Flywheel Speed Test", group = "Flywheel")
public class FlywheelSpeedTest extends OpMode {

    private Flywheel      flywheel;
    private VoltageSensor voltageSensor;
    private double        targetRPM = 0.0;

    private static final double STEP_COARSE = 100.0;
    private static final double STEP_FINE   = 10.0;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
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

        voltageSensor.update();
        flywheel.update();

        telemetry.addLine("Flywheel Speed Test");
        telemetry.addData("target RPM",  "%.1f", targetRPM);
        telemetry.addData("current RPM", "%.1f", flywheel.getRPM());
        telemetry.addData("state",       flywheel.getState());
        telemetry.addData("at speed",    flywheel.isAtSpeed());
        telemetry.addData("voltage",     "%.4f", voltageSensor.getVoltage());
        telemetry.addLine("DPAD UP/DOWN = ±100 RPM | DPAD RIGHT/LEFT = ±10 RPM | A = go | B = stop");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}