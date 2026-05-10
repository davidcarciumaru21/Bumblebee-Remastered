package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.constants.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp(name = "Flywheel kV Tuner", group = "Flywheel")
public class FlywheelKvTuner extends OpMode {

    private Flywheel      flywheel;
    private VoltageSensor voltageSensor;

    private static final double TUNING_POWER = 0.5;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
    }

    @Override
    public void loop() {

        flywheel.setPower(TUNING_POWER);

        voltageSensor.update();
        flywheel.update();

        double rpm            = flywheel.getRPM();
        double appliedVoltage = TUNING_POWER * voltageSensor.getVoltage();

        // kV = (appliedVoltage - kS) / rpm
        // represents how many volts are needed per RPM at steady state
        double kV = rpm > 0
                ? (appliedVoltage - SubsystemsConfig.Flywheel.KS) / rpm
                : 0.0;

        telemetry.addLine("Flywheel kV Tuner");
        telemetry.addData("power",           "%.4f", TUNING_POWER);
        telemetry.addData("voltage",         "%.4f", voltageSensor.getVoltage());
        telemetry.addData("rpm",             "%.1f", rpm);
        telemetry.addData("applied voltage", "%.4f", appliedVoltage);
        telemetry.addData("kS",              "%.4f", SubsystemsConfig.Flywheel.KS);
        telemetry.addData("kV (V/RPM)",      "%.6f", kV);
        telemetry.addLine("Wait for RPM to stabilize, then read kV");
        telemetry.update();
    }
}