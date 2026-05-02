package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Stopper;

@TeleOp(name = "Stopper Test TeleOp", group = "Stopper")
public class StopperTest extends OpMode {

    private Stopper stopper;

    @Override
    public void init() {
        stopper = new Stopper(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) stopper.open();
        if (gamepad1.bWasPressed()) stopper.close();

        stopper.update();

        telemetry.addData("state",    stopper.getState());
        telemetry.addData("position", stopper.getPosition());
        telemetry.addData("controls", "A = open | B = close");
        telemetry.update();
    }
}