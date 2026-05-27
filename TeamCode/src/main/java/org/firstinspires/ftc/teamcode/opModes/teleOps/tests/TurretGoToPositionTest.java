package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp(name = "Turret go to position Test", group = "Turret")
public class TurretGoToPositionTest extends OpMode {

    private VoltageSensor voltageSensor;
    private Turret Turret;
    private int targetAngle = 45;
    private int STEP = 5;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        Turret = new Turret(hardwareMap, voltageSensor);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) targetAngle += STEP;
        if (gamepad1.dpadDownWasPressed()) targetAngle -= STEP;

        Turret.setTargetAngle(targetAngle);
        Turret.update();

        telemetry.addData("state",    Turret.getState());
        telemetry.addData("position", Turret.getCurrentAngle());
        telemetry.addData("targetAngle", targetAngle);
        telemetry.update();
    }
}