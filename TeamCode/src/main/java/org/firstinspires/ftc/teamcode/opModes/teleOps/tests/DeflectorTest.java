package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;

@TeleOp(name = "Deflector Test TeleOp", group = "Deflector")
public class DeflectorTest extends OpMode {

    private Deflector deflector;

    private double currentPosition = SubsystemsConfig.Deflector.IDLE_POSITION;

    private static final double STEP = 0.01;

    @Override
    public void init() {
        deflector = new Deflector(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())   currentPosition = Math.min(1.0, currentPosition + STEP);
        if (gamepad1.dpadDownWasPressed()) currentPosition = Math.max(0.0, currentPosition - STEP);

        deflector.setPosition(currentPosition);

        deflector.update();

        telemetry.addData("state",            deflector.getState());
        telemetry.addData("current position", currentPosition);
        telemetry.addData("controls",         "DPAD UP = increment | DPAD DOWN = decrement");
        telemetry.update();
    }
}