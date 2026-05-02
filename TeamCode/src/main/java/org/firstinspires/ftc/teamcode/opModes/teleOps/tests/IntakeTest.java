package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Intake Test TeleOp", group = "Intake")
public class IntakeTest extends OpMode {

    private Intake intake;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if      (gamepad1.aWasPressed()) intake.pull();
        else if (gamepad1.bWasPressed()) intake.push();
        else if (gamepad1.xWasPressed()) intake.idle();

        intake.update();

        telemetry.addData("state",    intake.getState());
        telemetry.addData("controls", "A = pull | B = push | X = idle");
        telemetry.update();
    }
}