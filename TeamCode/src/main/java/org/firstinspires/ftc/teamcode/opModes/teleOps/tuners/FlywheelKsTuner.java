package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp(name = "Flywheel kS Tuner", group = "Flywheel")
public class FlywheelKsTuner extends OpMode {

    private Flywheel      flywheel;
    private VoltageSensor voltageSensor;
    private double        power = 0.0;

    private static final double STEP_COARSE = 0.01;
    private static final double STEP_FINE   = 0.001;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())      power += STEP_COARSE;
        if (gamepad1.dpadDownWasPressed())    power -= STEP_COARSE;
        if (gamepad1.rightBumperWasPressed()) power += STEP_FINE;
        if (gamepad1.leftBumperWasPressed())  power -= STEP_FINE;
        if (gamepad1.yWasPressed())           power  = 0.0;

        power = Math.max(0.0, Math.min(1.0, power));

        flywheel.setPower(power);

        voltageSensor.update();
        flywheel.update();

        // kS in volts = power * voltage, represents the minimum voltage
        // needed to overcome static friction and start the flywheel moving
        double ksVolts = power * voltageSensor.getVoltage();

        telemetry.addLine("Flywheel kS Tuner");
        telemetry.addData("power",      "%.4f", power);
        telemetry.addData("voltage",    "%.4f", voltageSensor.getVoltage());
        telemetry.addData("kS (volts)", "%.4f", ksVolts);
        telemetry.addData("rpm",        "%.1f", flywheel.getRPM());
        telemetry.addLine("DPAD UP/DOWN = ±0.01 | bumpers = ±0.001 | Y = reset");
        telemetry.update();
    }
}