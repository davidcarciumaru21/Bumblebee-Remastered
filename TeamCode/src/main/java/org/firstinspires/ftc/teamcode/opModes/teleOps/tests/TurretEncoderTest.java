package org.firstinspires.ftc.teamcode.opModes.teleOps.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Encoder Test", group = "Turret")
public class TurretEncoderTest extends OpMode {

    private Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {

        if      (gamepad1.right_trigger > 0.1) turret.setRawPower(gamepad1.right_trigger);
        else if (gamepad1.left_trigger  > 0.1) turret.setRawPower(-gamepad1.left_trigger);
        else                                   turret.setRawPower(0.0);

        turret.update();

        telemetry.addLine("Turret Encoder Test");
        telemetry.addData("angle (deg)",    "%.2f", turret.getCurrentAngle());
        telemetry.addData("encoder ticks",  turret.getEncoderTicks());
        telemetry.addData("controls",       "RT = clockwise | LT = counter-clockwise");
        telemetry.update();
    }
}