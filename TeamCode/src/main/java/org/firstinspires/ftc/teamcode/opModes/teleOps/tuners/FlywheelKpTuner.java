package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name = "Flywheel kP Tuner", group = "Flywheel")
public class FlywheelKpTuner extends OpMode {

    private Flywheel flywheel;

    private double kP    = SubsystemsConfig.Flywheel.KP;
    private int    scale = 0;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop() {

        // scale up / down
        if (gamepad1.dpadRightWasPressed()) scale++;
        if (gamepad1.dpadLeftWasPressed())  scale--;

        // modify kP
        if (gamepad1.dpadUpWasPressed())   kP += Math.pow(10, scale);
        if (gamepad1.dpadDownWasPressed()) kP -= Math.pow(10, scale);

        kP = Math.max(0.0, kP);

        flywheel.setKP(kP);

        if (gamepad1.aWasPressed()) flywheel.setRPM(3000);
        if (gamepad1.bWasPressed()) flywheel.stop();

        flywheel.update();

        telemetry.addLine("Flywheel kP Tuner");
        telemetry.addLine("Copy final kP value to SubsystemsConfig.Flywheel.KP");
        telemetry.addData("kP",          "%.6f", kP);
        telemetry.addData("scale",       "10^" + scale + " = " + Math.pow(10, scale));
        telemetry.addData("target RPM",  "%.1f", flywheel.getTargetRPM());
        telemetry.addData("current RPM", "%.1f", flywheel.getRPM());
        telemetry.addData("state",       flywheel.getState());
        telemetry.addLine("DPAD UP/DOWN = modify kP | DPAD RIGHT/LEFT = scale | A = 3000 RPM | B = stop");
        telemetry.update();
    }
}