package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret go to position TeleOp", group = "Turret")
public class TurretGoToPositionTest extends OpMode {

    private Turret Turret;

    @Override
    public void init() {
        Turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        Turret.setTargetAngle(45);
        Turret.update();

        telemetry.addData("state",    Turret.getState());
        telemetry.addData("position", Turret.getCurrentAngle());
        telemetry.update();
    }
}