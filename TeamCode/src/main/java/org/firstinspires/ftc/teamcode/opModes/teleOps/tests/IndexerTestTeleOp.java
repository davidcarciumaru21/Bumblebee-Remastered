package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "Indexer Test TeleOp", group = "Indexer")
public class IndexerTestTeleOp extends OpMode {

    private Indexer indexer;
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;

    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
    }

    @Override
    public void loop() {
        if      (gamepad1.a && !previousA) indexer.pull();
        else if (gamepad1.b && !previousB) indexer.push();
        else if (gamepad1.x && !previousX) indexer.idle();

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;

        indexer.update();

        telemetry.addData("state",    indexer.getState());
        telemetry.addData("controls", "A = pull | B = push | X = idle");
        telemetry.update();
    }
}