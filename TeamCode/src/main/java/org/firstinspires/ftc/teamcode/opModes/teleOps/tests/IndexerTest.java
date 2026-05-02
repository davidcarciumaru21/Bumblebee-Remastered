package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "Indexer Test TeleOp", group = "Indexer")
public class IndexerTest extends OpMode {

    private Indexer indexer;

    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
    }

    @Override
    public void loop() {
        if      (gamepad1.aWasPressed()) indexer.pull();
        else if (gamepad1.bWasPressed()) indexer.push();
        else if (gamepad1.xWasPressed()) indexer.idle();

        indexer.update();

        telemetry.addData("state",    indexer.getState());
        telemetry.addData("controls", "A = pull | B = push | X = idle");
        telemetry.update();
    }
}