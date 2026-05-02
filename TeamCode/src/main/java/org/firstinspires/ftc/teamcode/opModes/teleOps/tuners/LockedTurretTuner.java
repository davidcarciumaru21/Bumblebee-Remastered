package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.LockedTurret;

@TeleOp(name = "Locked Turret Test TeleOp", group = "Turret")
public class LockedTurretTuner extends OpMode {

    private LockedTurret lockedTurret;

    private double currentPosition = SubsystemsConfig.LockedTurret.LOCKED_POSITION;

    private static final double STEP = 0.01;

    @Override
    public void init() {
        lockedTurret = new LockedTurret(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_right) currentPosition = Math.min(1.0, currentPosition + STEP);
        if (gamepad1.dpad_left)  currentPosition = Math.max(0.0, currentPosition - STEP);

        lockedTurret.setPosition(currentPosition);
        lockedTurret.update();

        telemetry.addData("state",            lockedTurret.getState());
        telemetry.addData("current position", currentPosition);
        telemetry.addData("controls",         "DPAD RIGHT = increment | DPAD LEFT = decrement");
        telemetry.update();
    }
}