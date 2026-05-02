package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Intake Test TeleOp", group = "Intake")
public class IntakeTestTeleOp extends OpMode {

    private Intake intake;
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if      (gamepad1.a && !previousA) intake.pull();
        else if (gamepad1.b && !previousB) intake.push();
        else if (gamepad1.x && !previousX) intake.idle();

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;

        intake.update();

        telemetry.addData("state",    intake.getState());
        telemetry.addData("controls", "A = pull | B = push | X = idle");
        telemetry.update();
    }
}