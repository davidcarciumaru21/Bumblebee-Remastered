package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp(name = "Turret Min Power Tuner", group = "Turret")
public class TurretMinPowerTuner extends OpMode {

    private VoltageSensor voltageSensor;
    private Turret turret;

    private double currentPower = 0.0;

    private static final double STEP_COARSE = 0.01;
    private static final double STEP_FINE   = 0.001;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        turret = new Turret(hardwareMap, voltageSensor);
    }

    private double lastPower = -1.0;

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())      currentPower += STEP_COARSE;
        if (gamepad1.dpadDownWasPressed())    currentPower -= STEP_COARSE;
        if (gamepad1.rightBumperWasPressed()) currentPower += STEP_FINE;
        if (gamepad1.leftBumperWasPressed())  currentPower -= STEP_FINE;
        if (gamepad1.yWasPressed())           currentPower  = 0.0;

        currentPower = Math.max(0.0, Math.min(1.0, currentPower));

        if (currentPower != lastPower) {
            turret.setRawPower(currentPower);
            lastPower = currentPower;
        }

        voltageSensor.update();

        double minPowerVolts = currentPower * voltageSensor.getVoltage();

        telemetry.addLine("Turret Min Power Tuner");
        telemetry.addLine("Increase power until turret starts moving — copy MIN_POWER_VOLTS to SubsystemsConfig.Turret.MIN_POWER_VOLTS");
        telemetry.addData("voltage",         "%.4f", voltageSensor.getVoltage());
        telemetry.addData("MIN_POWER_VOLTS", "%.4f", minPowerVolts);
        telemetry.addLine("DPAD UP/DOWN = ±0.01 | bumpers = ±0.001 | Y = reset");
        telemetry.update();
    }
}