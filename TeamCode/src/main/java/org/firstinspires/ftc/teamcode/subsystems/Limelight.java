package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;

/**
 * Limelight3A wrapper for target data and field-pose relocalization.
 * Call {@link #update()} once per loop before reading cached values.
 */
public class Limelight implements Subsystem {

    private final Limelight3A limelight;

    private LLResult latestResult;
    private Pose latestPose;
    private Pose3D latestFullPose;

    private double lastYaw = 0.0;
    private double lastTargetArea = 0.0;
    private double filteredX = SubsystemsConfig.Limelight.PEDRO_FIELD_CENTER_OFFSET_INCHES;
    private double filteredY = SubsystemsConfig.Limelight.PEDRO_FIELD_CENTER_OFFSET_INCHES;
    private boolean hasTarget = false;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, SubsystemsConfig.Limelight.DEFAULT_PIPELINE);
    }

    public Limelight(HardwareMap hardwareMap, int pipelineIndex) {
        this.limelight = hardwareMap.get(
                Limelight3A.class,
                SubsystemsConfig.Limelight.HARDWARE_MAP_NAME
        );
        this.limelight.pipelineSwitch(pipelineIndex);
        this.limelight.start();
    }

    @Override
    public void update() {
        latestResult = limelight.getLatestResult();
        hasTarget = latestResult != null && latestResult.isValid();

        if (!hasTarget) {
            latestPose = null;
            latestFullPose = null;
            return;
        }

        lastYaw = latestResult.getTx();
        lastTargetArea = latestResult.getTa();
        latestFullPose = latestResult.getBotpose();
        latestPose = convertToPedroPose(latestFullPose);
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void stop() {
        limelight.stop();
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getYaw() {
        return lastYaw;
    }

    public double getTargetArea() {
        return lastTargetArea;
    }

    public Pose getPose() {
        return latestPose;
    }

    public Pose3D getFullPose() {
        return latestFullPose;
    }

    private Pose convertToPedroPose(Pose3D botPose) {
        if (botPose == null) {
            return null;
        }

        double zInches = botPose.getPosition().z * SubsystemsConfig.Limelight.METERS_TO_INCHES;
        if (Math.abs(zInches) >= SubsystemsConfig.Limelight.MAX_VALID_Z_ERROR_INCHES) {
            return null;
        }

        double xInches = botPose.getPosition().x * SubsystemsConfig.Limelight.METERS_TO_INCHES;
        double yInches = botPose.getPosition().y * SubsystemsConfig.Limelight.METERS_TO_INCHES;

        double xPedro = xInches + SubsystemsConfig.Limelight.PEDRO_FIELD_CENTER_OFFSET_INCHES;
        double yPedro = Math.abs(yInches) + SubsystemsConfig.Limelight.PEDRO_FIELD_CENTER_OFFSET_INCHES;

        filteredX += SubsystemsConfig.Limelight.FILTER_ALPHA * (xPedro - filteredX);
        filteredY += SubsystemsConfig.Limelight.FILTER_ALPHA * (yPedro - filteredY);

        double headingPedro = -Math.toRadians(botPose.getOrientation().getYaw());

        return new Pose(filteredX, filteredY, headingPedro);
    }
}
