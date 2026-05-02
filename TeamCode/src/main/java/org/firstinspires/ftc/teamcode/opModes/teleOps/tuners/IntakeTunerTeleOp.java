package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Intake Tuner TeleOp", group = "Intake")
public class IntakeTunerTeleOp extends OpMode {

    private Intake intake;

    private double currentPower = 0.0;
    private double savedPull    = SubsystemsConfig.Intake.PULL_POWER;
    private double savedPush    = SubsystemsConfig.Intake.PUSH_POWER;
    private double savedIdle    = SubsystemsConfig.Intake.IDLE_POWER;

    private static final double STEP = 0.01;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {

        // increment / decrement current power
        if (gamepad1.dpadUpWasPressed())   currentPower = Math.min(1.0,  currentPower + STEP);
        if (gamepad1.dpadDownWasPressed()) currentPower = Math.max(-1.0, currentPower - STEP);

        // save current power as pull / push / idle
        if (gamepad1.aWasPressed()) savedPull = currentPower;
        if (gamepad1.bWasPressed()) savedPush = currentPower;
        if (gamepad1.xWasPressed()) savedIdle = currentPower;

        // activate saved powers
        if      (gamepad1.right_trigger > 0.5) intake.setPower(savedPull);
        else if (gamepad1.left_trigger  > 0.5) intake.setPower(savedPush);
        else if (gamepad1.yWasPressed())        intake.setPower(savedIdle);

        intake.update();

        telemetry.addData("state",         intake.getState());
        telemetry.addData("current power", currentPower);
        telemetry.addData("saved pull",    savedPull);
        telemetry.addData("saved push",    savedPush);
        telemetry.addData("saved idle",    savedIdle);
        telemetry.addData("controls",      "DPAD UP/DOWN = tune | A = save pull | B = save push | X = save idle | RT = pull | LT = push | Y = idle");
        telemetry.update();
    }
}