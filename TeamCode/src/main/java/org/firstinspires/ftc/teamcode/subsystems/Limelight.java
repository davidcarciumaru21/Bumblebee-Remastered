package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;

import java.util.List;

/**
 * Limelight3A wrapper for target data and field-pose relocalization.
 * Call {@link #update()} once per loop before reading cached values.
 */
public class Limelight implements Subsystem {

    public enum BallColor {
        GREEN(SubsystemsConfig.Limelight.GREEN_BALL_PIPELINE),
        PURPLE(SubsystemsConfig.Limelight.PURPLE_BALL_PIPELINE);

        private final int pipelineIndex;

        BallColor(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public int getPipelineIndex() {
            return pipelineIndex;
        }

        private static BallColor fromPipeline(int pipelineIndex) {
            for (BallColor color : values()) {
                if (color.pipelineIndex == pipelineIndex) {
                    return color;
                }
            }

            return null;
        }
    }

    public static class BallTarget {
        private final BallColor color;
        private final double xDegrees;
        private final double yDegrees;
        private final double area;
        private final long timestampNanos;

        private BallTarget(BallColor color, double xDegrees, double yDegrees, double area) {
            this.color = color;
            this.xDegrees = xDegrees;
            this.yDegrees = yDegrees;
            this.area = area;
            this.timestampNanos = System.nanoTime();
        }

        public BallColor getColor() {
            return color;
        }

        public double getXDegrees() {
            return xDegrees;
        }

        public double getYDegrees() {
            return yDegrees;
        }

        public double getArea() {
            return area;
        }

        public double getAgeMs() {
            return (System.nanoTime() - timestampNanos) / 1_000_000.0;
        }

        public boolean isRecent(double maxAgeMs) {
            return getAgeMs() <= maxAgeMs;
        }
    }

    private final Limelight3A limelight;

    private LLResult latestResult;
    private Pose latestPose;
    private Pose3D latestFullPose;
    private BallTarget latestBallTarget;
    private BallTarget latestGreenBall;
    private BallTarget latestPurpleBall;

    private double lastYaw = 0.0;
    private double lastTargetArea = 0.0;
    private double filteredX = SubsystemsConfig.Limelight.PEDRO_FIELD_CENTER_OFFSET_INCHES;
    private double filteredY = SubsystemsConfig.Limelight.PEDRO_FIELD_CENTER_OFFSET_INCHES;
    private boolean hasTarget = false;
    private int activePipelineIndex = -1;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, SubsystemsConfig.Limelight.DEFAULT_PIPELINE);
    }

    public Limelight(HardwareMap hardwareMap, int pipelineIndex) {
        this.limelight = hardwareMap.get(
                Limelight3A.class,
                SubsystemsConfig.Limelight.HARDWARE_MAP_NAME
        );
        setPipeline(pipelineIndex);
        this.limelight.start();
    }

    @Override
    public void update() {
        latestResult = limelight.getLatestResult();
        hasTarget = latestResult != null && latestResult.isValid();
        latestBallTarget = null;

        if (!hasTarget) {
            latestPose = null;
            latestFullPose = null;
            return;
        }

        lastYaw = latestResult.getTx();
        lastTargetArea = latestResult.getTa();
        latestFullPose = latestResult.getBotpose();
        latestPose = convertToPedroPose(latestFullPose);
        latestBallTarget = getLargestBallTarget(
                BallColor.fromPipeline(activePipelineIndex),
                latestResult.getColorResults()
        );
    }

    public void setPipeline(int pipelineIndex) {
        if (activePipelineIndex != pipelineIndex) {
            limelight.pipelineSwitch(pipelineIndex);
        }

        activePipelineIndex = pipelineIndex;
    }

    public void setBallPipeline(BallColor color) {
        setPipeline(color.getPipelineIndex());
    }

    public void beginAlternatingBallTracking() {
        latestBallTarget = null;
        latestGreenBall = null;
        latestPurpleBall = null;
        setBallPipeline(BallColor.GREEN);
    }

    public void updateAlternatingBallTracking() {
        update();

        if (latestBallTarget != null) {
            if (latestBallTarget.getColor() == BallColor.GREEN) {
                latestGreenBall = latestBallTarget;
            } else if (latestBallTarget.getColor() == BallColor.PURPLE) {
                latestPurpleBall = latestBallTarget;
            }
        }

        BallColor activeColor = BallColor.fromPipeline(activePipelineIndex);
        setBallPipeline(activeColor == BallColor.GREEN ? BallColor.PURPLE : BallColor.GREEN);
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

    public int getActivePipelineIndex() {
        return activePipelineIndex;
    }

    public BallTarget getLatestBallTarget() {
        return latestBallTarget;
    }

    public BallTarget getBallTarget(BallColor color) {
        return color == BallColor.GREEN ? latestGreenBall : latestPurpleBall;
    }

    public BallTarget getRecentBallTarget(BallColor color, double maxAgeMs) {
        BallTarget target = getBallTarget(color);
        return target != null && target.isRecent(maxAgeMs) ? target : null;
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

    private BallTarget getLargestBallTarget(BallColor color, List<LLResultTypes.ColorResult> targets) {
        if (color == null || targets == null) {
            return null;
        }

        LLResultTypes.ColorResult bestTarget = null;
        double bestArea = 0.0;

        for (LLResultTypes.ColorResult target : targets) {
            if (target.getTargetArea() > bestArea) {
                bestArea = target.getTargetArea();
                bestTarget = target;
            }
        }

        if (bestTarget == null) {
            return null;
        }

        return new BallTarget(
                color,
                bestTarget.getTargetXDegrees(),
                bestTarget.getTargetYDegrees(),
                bestTarget.getTargetArea()
        );
    }
}
