package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deflector;

@TeleOp(name = "Deflector Angle Test TeleOp", group = "Deflector")
public class DeflectorAngleTest extends OpMode {

    private Deflector deflector;

    private double currentAngle = 40.0;

    private static final double STEP = 2.0; // degrees

    @Override
    public void init() {
        deflector = new Deflector(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())   currentAngle += STEP;
        if (gamepad1.dpadDownWasPressed()) currentAngle -= STEP;

        deflector.setAngleInDegrees(currentAngle);

        deflector.update();

        telemetry.addData("state",         deflector.getState());
        telemetry.addData("angle (deg)",   currentAngle);
        telemetry.addData("position",      deflector.getTargetPosition());
        telemetry.addData("controls",      "DPAD UP = +1° | DPAD DOWN = -1°");
        telemetry.update();
    }
}