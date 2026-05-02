package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name = "Flywheel Ramp Test", group = "Flywheel")
public class FlywheelRampTest extends OpMode {

    private Flywheel flywheel;
    private ElapsedTime totalTimer = new ElapsedTime();

    private static final double TARGET_RPM = 3000;

    private boolean reachedSpeed  = false;
    private double  timeToSpeedMs = 0.0;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void start() {
        totalTimer.reset();
        reachedSpeed  = false;
        timeToSpeedMs = 0.0;
        flywheel.setRPM(TARGET_RPM);
    }

    @Override
    public void loop() {
        flywheel.update();

        double rpm = flywheel.getRPM();

        if (!reachedSpeed && Math.abs(rpm - TARGET_RPM) <= SubsystemsConfig.Flywheel.AT_SPEED_TOLERANCE) {
            timeToSpeedMs = totalTimer.milliseconds();
            reachedSpeed  = true;
        }

        telemetry.addLine("Flywheel Ramp Test");
        telemetry.addData("target RPM",       TARGET_RPM);
        telemetry.addData("current RPM",      "%.1f", rpm);
        telemetry.addData("state",            flywheel.getState());
        telemetry.addData("reached speed",    reachedSpeed);
        telemetry.addData("time to speed ms", "%.1f", timeToSpeedMs);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}