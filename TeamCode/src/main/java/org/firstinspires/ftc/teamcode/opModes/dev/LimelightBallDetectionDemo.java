package org.firstinspires.ftc.teamcode.opModes.dev;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;

import java.util.List;

@TeleOp(name = "Limelight Ball Detection Demo", group = "dev")
public class LimelightBallDetectionDemo extends OpMode {

    private Limelight3A limelight;
    private int pipeline = SubsystemsConfig.Limelight.GREEN_BALL_PIPELINE;

    @Override
    public void init() {
        limelight = hardwareMap.get(
                Limelight3A.class,
                SubsystemsConfig.Limelight.HARDWARE_MAP_NAME
        );
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Limelight ready");
        telemetry.addLine("A = green pipeline, B = purple pipeline");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            setPipeline(SubsystemsConfig.Limelight.GREEN_BALL_PIPELINE);
        }

        if (gamepad1.bWasPressed()) {
            setPipeline(SubsystemsConfig.Limelight.PURPLE_BALL_PIPELINE);
        }

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        telemetry.addLine("=== Limelight Ball Detection ===");
        telemetry.addData("selected pipeline", pipeline);
        telemetry.addData("LL pipeline", "%d / %s", status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("LL fps", "%.0f", status.getFps());

        if (result == null || !result.isValid()) {
            telemetry.addData("ball", "not detected");
            telemetry.update();
            return;
        }

        LLResultTypes.ColorResult bestBall = getLargestColorTarget(result.getColorResults());

        if (bestBall == null) {
            telemetry.addData("ball", "not detected");
            telemetry.addData("tx", "%.2f", result.getTx());
            telemetry.addData("ty", "%.2f", result.getTy());
            telemetry.addData("ta", "%.2f", result.getTa());
            telemetry.update();
            return;
        }

        double tx = bestBall.getTargetXDegrees();
        double ty = bestBall.getTargetYDegrees();
        double area = bestBall.getTargetArea();

        telemetry.addData("ball", pipeline == SubsystemsConfig.Limelight.GREEN_BALL_PIPELINE ? "green" : "purple");
        telemetry.addData("tx deg", "%.2f", tx);
        telemetry.addData("ty deg", "%.2f", ty);
        telemetry.addData("area", "%.2f", area);
        telemetry.addData("avoid", tx >= 0.0 ? "strafe left" : "strafe right");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private void setPipeline(int pipeline) {
        this.pipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    private LLResultTypes.ColorResult getLargestColorTarget(List<LLResultTypes.ColorResult> targets) {
        LLResultTypes.ColorResult bestTarget = null;
        double bestArea = 0.0;

        for (LLResultTypes.ColorResult target : targets) {
            if (target.getTargetArea() > bestArea) {
                bestArea = target.getTargetArea();
                bestTarget = target;
            }
        }

        return bestTarget;
    }
}
