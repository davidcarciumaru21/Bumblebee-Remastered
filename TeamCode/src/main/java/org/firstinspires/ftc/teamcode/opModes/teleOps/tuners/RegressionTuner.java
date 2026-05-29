package org.firstinspires.ftc.teamcode.opModes.teleOps.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

/**
 * Regression Tuner — manually controls flywheel RPM, deflector angle, stopper and indexer.
 * Use this to find the correct RPM and deflector angle for each shooting distance.
 *
 * Controls:
 * - DPAD UP/DOWN    — flywheel RPM ±100
 * - DPAD RIGHT/LEFT — deflector angle ±1°
 * - A               — toggle stopper open/close
 * - B               — toggle indexer on/off
 */
@TeleOp(name = "Regression Tuner", group = "tuners")
public class RegressionTuner extends OpMode {

    private VoltageSensor voltageSensor;
    private Flywheel      flywheel;
    private Deflector     deflector;
    private Indexer       indexer;
    private Stopper       stopper;
    private Intake        intake;

    private double  targetRPM      = 0.0;
    private double  deflectorAngle = 70.0;
    private boolean stopperOpen    = false;
    private boolean indexerOn      = false;

    private static final double RPM_STEP      = 100.0;
    private static final double DEFLECTOR_STEP = 1.0;

    @Override
    public void init() {
        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
        deflector     = new Deflector(hardwareMap);
        indexer       = new Indexer(hardwareMap);
        stopper       = new Stopper(hardwareMap);
        intake        = new Intake(hardwareMap);
    }

    @Override
    public void loop() {

        // flywheel RPM
        if (gamepad1.dpadUpWasPressed())   targetRPM += RPM_STEP;
        if (gamepad1.dpadDownWasPressed()) targetRPM -= RPM_STEP;
        targetRPM = Math.max(0.0, targetRPM);

        // deflector angle
        if (gamepad1.dpadRightWasPressed()) deflectorAngle += DEFLECTOR_STEP;
        if (gamepad1.dpadLeftWasPressed())  deflectorAngle -= DEFLECTOR_STEP;

        // stopper toggle
        if (gamepad1.aWasPressed()) {
            stopperOpen = !stopperOpen;
            if (stopperOpen) stopper.open();
            else             stopper.close();
        }

        // indexer toggle
        if (gamepad1.bWasPressed()) {
            indexerOn = !indexerOn;
            if (indexerOn) indexer.pull();
            else           indexer.idle();
        }

        // apply targets
        if (targetRPM > 0) flywheel.setRPM(targetRPM);
        else                flywheel.stop();

        deflector.setAngleInDegrees(deflectorAngle);
        intake.pull();

        // update subsystems
        voltageSensor.update();
        flywheel.update();
        deflector.update();
        indexer.update();
        stopper.update();
        intake.update();

        // telemetry
        telemetry.addLine("=== Regression Tuner ===");
        telemetry.addData("target RPM",      "%.0f",  targetRPM);
        telemetry.addData("current RPM",     "%.1f",  flywheel.getRPM());
        telemetry.addData("flywheel state",  flywheel.getState());
        telemetry.addData("at speed",        flywheel.isAtSpeed());
        telemetry.addData("deflector angle", "%.1f°", deflectorAngle);
        telemetry.addData("stopper",         stopperOpen ? "OPEN" : "CLOSED");
        telemetry.addData("indexer",         indexerOn   ? "ON"   : "OFF");
        telemetry.addData("voltage",         "%.3fV",    voltageSensor.getVoltage());
        telemetry.addLine("DPAD UP/DOWN = RPM ±100 | DPAD RIGHT/LEFT = angle ±1° | A = stopper | B = indexer");
        telemetry.update();
    }
}