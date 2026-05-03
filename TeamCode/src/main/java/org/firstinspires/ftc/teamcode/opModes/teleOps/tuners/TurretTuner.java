package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Tuner", group = "Turret")
public class TurretTuner extends OpMode {

    private Turret turret;

    private double brakeDistance = SubsystemsConfig.Turret.BRAKE_DISTANCE;
    private double deadZone      = SubsystemsConfig.Turret.DEAD_ZONE;

    private static final double BRAKE_STEP = 1.0;   // degrees
    private static final double DEAD_STEP  = 0.1;   // degrees

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {

        // modify BRAKE_DISTANCE
        if (gamepad1.dpadUpWasPressed())    brakeDistance += BRAKE_STEP;
        if (gamepad1.dpadDownWasPressed())  brakeDistance -= BRAKE_STEP;

        // modify DEAD_ZONE
        if (gamepad1.dpadRightWasPressed()) deadZone += DEAD_STEP;
        if (gamepad1.dpadLeftWasPressed())  deadZone -= DEAD_STEP;

        brakeDistance = Math.max(1.0, brakeDistance);
        deadZone      = Math.max(0.1, deadZone);

        // apply tuned values live
        turret.setBrakeDistance(brakeDistance);
        turret.setDeadZone(deadZone);

        // set targets
        if (gamepad1.aWasPressed()) turret.setTargetAngle(45);
        if (gamepad1.bWasPressed()) turret.setTargetAngle(-45);
        if (gamepad1.yWasPressed()) turret.setTargetAngle(0);

        turret.update();

        telemetry.addLine("Turret Tuner");
        telemetry.addLine("Copy final values to SubsystemsConfig.Turret");
        telemetry.addData("BRAKE_DISTANCE", "%.1f", brakeDistance);
        telemetry.addData("DEAD_ZONE",      "%.2f", deadZone);
        telemetry.addData("target angle",   "%.2f", turret.getTargetAngle());
        telemetry.addData("current angle",  "%.2f", turret.getCurrentAngle());
        telemetry.addData("state",          turret.getState());
        telemetry.addData("voltage",        "%.4f", turret.getFilteredVoltage());
        telemetry.addLine("DPAD UP/DOWN = BRAKE_DISTANCE ±1 | DPAD RIGHT/LEFT = DEAD_ZONE ±0.1");
        telemetry.addLine("A = +45° | B = -45° | Y = 0°");
        telemetry.update();
    }
}