package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Min Power Tuner", group = "Turret")
public class TurretMinPowerTuner extends OpMode {

    private Turret turret;

    private double currentPower = 0.0;

    private static final double STEP_COARSE = 0.01;
    private static final double STEP_FINE   = 0.001;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed())      currentPower += STEP_COARSE;
        if (gamepad1.dpadDownWasPressed())    currentPower -= STEP_COARSE;
        if (gamepad1.rightBumperWasPressed()) currentPower += STEP_FINE;
        if (gamepad1.leftBumperWasPressed())  currentPower -= STEP_FINE;
        if (gamepad1.yWasPressed())           currentPower  = 0.0;

        currentPower = Math.max(0.0, Math.min(1.0, currentPower));

        turret.setRawPower(currentPower);
        turret.update();

        // MIN_POWER_VOLTS = currentPower * filteredVoltage
        double minPowerVolts = currentPower * turret.getFilteredVoltage();

        telemetry.addLine("Turret Min Power Tuner");
        telemetry.addLine("Increase power until turret starts moving — copy MIN_POWER_VOLTS to SubsystemsConfig.Turret.MIN_POWER_VOLTS");
        telemetry.addData("current power",    "%.4f", currentPower);
        telemetry.addData("voltage",          "%.4f", turret.getFilteredVoltage());
        telemetry.addData("MIN_POWER_VOLTS",  "%.4f", minPowerVolts);
        telemetry.addData("angle (deg)",      "%.2f",  turret.getCurrentAngle());
        telemetry.addData("encoder ticks",    turret.getEncoderTicks());
        telemetry.addLine("DPAD UP/DOWN = ±0.01 | bumpers = ±0.001 | Y = reset");
        telemetry.update();
    }
}