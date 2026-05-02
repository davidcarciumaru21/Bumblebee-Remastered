package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

@TeleOp(name = "Stopper Test TeleOp", group = "Stopper")
public class StopperTuner extends OpMode {

    private Stopper stopper;

    private double savedOpenPosition   = SubsystemsConfig.Stopper.OPEN_POSITION;
    private double savedClosedPosition = SubsystemsConfig.Stopper.CLOSED_POSITION;
    private double currentPosition     = SubsystemsConfig.Stopper.OPEN_POSITION;

    private static final double STEP = 0.01;

    @Override
    public void init() {
        stopper = new Stopper(hardwareMap);
    }

    @Override
    public void loop() {

        // increment / decrement current position
        if (gamepad1.dpadUpWasPressed())   currentPosition = Math.min(1.0, currentPosition + STEP);
        if (gamepad1.dpadDownWasPressed()) currentPosition = Math.max(0.0, currentPosition - STEP);

        // apply current position to servo
        stopper.setPosition(currentPosition);

        // save current position as open or closed
        if (gamepad1.aWasPressed()) savedOpenPosition   = currentPosition;
        if (gamepad1.bWasPressed()) savedClosedPosition = currentPosition;

        // go to saved positions
        if (gamepad1.xWasPressed()) {
            currentPosition = savedOpenPosition;
            stopper.setPosition(currentPosition);
        }

        if (gamepad1.yWasPressed()) {
            currentPosition = savedClosedPosition;
            stopper.setPosition(currentPosition);
        }

        stopper.update();

        telemetry.addData("state",            stopper.getState());
        telemetry.addData("current position", currentPosition);
        telemetry.addData("saved open",       savedOpenPosition);
        telemetry.addData("saved closed",     savedClosedPosition);
        telemetry.addData("controls",         "DPAD UP/DOWN = tune | A = save open | B = save closed | X = go open | Y = go closed");
        telemetry.update();
    }
}