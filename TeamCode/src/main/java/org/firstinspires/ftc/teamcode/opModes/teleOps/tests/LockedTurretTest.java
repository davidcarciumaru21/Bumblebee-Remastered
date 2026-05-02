package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LockedTurret;

@TeleOp(name = "Locked Turret TeleOp", group = "Turret")
public class LockedTurretTest extends OpMode {

    private LockedTurret lockedTurret;

    @Override
    public void init() {
        lockedTurret = new LockedTurret(hardwareMap);
    }

    @Override
    public void loop() {
        lockedTurret.lock();
        lockedTurret.update();

        telemetry.addData("state",    lockedTurret.getState());
        telemetry.addData("position", lockedTurret.getTargetPosition());
        telemetry.update();
    }
}