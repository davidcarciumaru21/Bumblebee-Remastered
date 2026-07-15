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

    public enum MazeColumn {
        LEFT(0),
        CENTER(1),
        RIGHT(2);

        private final int index;

        MazeColumn(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

        private static MazeColumn fromIndex(int index) {
            for (MazeColumn column : values()) {
                if (column.index == index) {
                    return column;
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

    public static class MazeRowScan {
        private final int rowIndex;
        private final BallColor color;
        private final boolean[] occupiedColumns;
        private final double[] columnAreas;
        private final double[] columnXDegrees;
        private final int acceptedTargetCount;
        private final long timestampNanos;

        private MazeRowScan(
                int rowIndex,
                BallColor color,
                boolean[] occupiedColumns,
                double[] columnAreas,
                double[] columnXDegrees,
                int acceptedTargetCount
        ) {
            this.rowIndex = rowIndex;
            this.color = color;
            this.occupiedColumns = occupiedColumns.clone();
            this.columnAreas = columnAreas.clone();
            this.columnXDegrees = columnXDegrees.clone();
            this.acceptedTargetCount = acceptedTargetCount;
            this.timestampNanos = System.nanoTime();
        }

        public int getRowIndex() {
            return rowIndex;
        }

        public BallColor getColor() {
            return color;
        }

        public boolean isColumnOccupied(int columnIndex) {
            validateMazeColumnIndex(columnIndex);
            return occupiedColumns[columnIndex];
        }

        public double getColumnArea(int columnIndex) {
            validateMazeColumnIndex(columnIndex);
            return columnAreas[columnIndex];
        }

        public double getColumnXDegrees(int columnIndex) {
            validateMazeColumnIndex(columnIndex);
            return columnXDegrees[columnIndex];
        }

        public int getAcceptedTargetCount() {
            return acceptedTargetCount;
        }

        public int getOccupiedColumnCount() {
            int count = 0;

            for (boolean occupied : occupiedColumns) {
                if (occupied) {
                    count++;
                }
            }

            return count;
        }

        public int getOpenColumnIndex() {
            int openColumn = -1;

            for (int columnIndex = 0; columnIndex < occupiedColumns.length; columnIndex++) {
                if (!occupiedColumns[columnIndex]) {
                    if (openColumn != -1) {
                        return -1;
                    }

                    openColumn = columnIndex;
                }
            }

            return openColumn;
        }

        public MazeColumn getOpenColumn() {
            return MazeColumn.fromIndex(getOpenColumnIndex());
        }

        public boolean hasOpenColumn() {
            return getOpenColumnIndex() != -1;
        }

        public boolean isReliable() {
            return getOccupiedColumnCount() == SubsystemsConfig.Limelight.Maze.COLUMN_COUNT - 1
                    && hasOpenColumn();
        }

        public double getAgeMs() {
            return (System.nanoTime() - timestampNanos) / 1_000_000.0;
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

    public void setMazeRowPipeline(int rowIndex) {
        setBallPipeline(getMazeRowColor(rowIndex));
    }

    public MazeRowScan updateMazeRowScan(int rowIndex) {
        update();
        return getMazeRowScan(rowIndex);
    }

    public MazeRowScan updateMazeRowScan(int rowIndex, Pose robotPose) {
        update();
        return getMazeRowScan(rowIndex, robotPose);
    }

    public MazeRowScan getMazeRowScan(int rowIndex) {
        return getMazeRowScan(rowIndex, null);
    }

    public MazeRowScan getMazeRowScan(int rowIndex, Pose robotPose) {
        validateMazeRowIndex(rowIndex);

        List<LLResultTypes.ColorResult> targets = latestResult != null && latestResult.isValid()
                ? latestResult.getColorResults()
                : null;

        return buildMazeRowScan(rowIndex, targets, robotPose);
    }

    public static BallColor getMazeRowColor(int rowIndex) {
        validateMazeRowIndex(rowIndex);
        return rowIndex < SubsystemsConfig.Limelight.Maze.GREEN_ROW_COUNT
                ? BallColor.GREEN
                : BallColor.PURPLE;
    }

    public static Pose getMazeCellPose(int rowIndex, int columnIndex) {
        validateMazeRowIndex(rowIndex);
        validateMazeColumnIndex(columnIndex);

        double heading = Math.toRadians(SubsystemsConfig.Limelight.Maze.HEADING_DEGREES);
        double forwardX = Math.cos(heading);
        double forwardY = Math.sin(heading);
        double leftX = -Math.sin(heading);
        double leftY = Math.cos(heading);
        double forwardOffset = rowIndex * SubsystemsConfig.Limelight.Maze.ROW_SPACING_INCHES;
        double lateralOffset = (1 - columnIndex) * SubsystemsConfig.Limelight.Maze.COLUMN_SPACING_INCHES;

        double x = SubsystemsConfig.Limelight.Maze.ORIGIN_X_INCHES
                + forwardOffset * forwardX
                + lateralOffset * leftX;
        double y = SubsystemsConfig.Limelight.Maze.ORIGIN_Y_INCHES
                + forwardOffset * forwardY
                + lateralOffset * leftY;

        return new Pose(x, y, heading);
    }

    public static Pose getMazeScanPose(int rowIndex, int columnIndex) {
        validateMazeRowIndex(rowIndex);
        validateMazeColumnIndex(columnIndex);

        double heading = Math.toRadians(SubsystemsConfig.Limelight.Maze.HEADING_DEGREES);
        double forwardX = Math.cos(heading);
        double forwardY = Math.sin(heading);
        double leftX = -Math.sin(heading);
        double leftY = Math.cos(heading);
        double scanForwardOffset = rowIndex * SubsystemsConfig.Limelight.Maze.ROW_SPACING_INCHES
                - SubsystemsConfig.Limelight.Maze.SCAN_X_OFFSET_INCHES;

        if (rowIndex > 0) {
            double previousRowClearOffset = (rowIndex - 1) * SubsystemsConfig.Limelight.Maze.ROW_SPACING_INCHES
                    + SubsystemsConfig.RobotDimensions.LENGTH / 2.0
                    + SubsystemsConfig.Limelight.Maze.PASSED_ROW_REAR_CLEARANCE_INCHES;
            scanForwardOffset = Math.max(scanForwardOffset, previousRowClearOffset);
        }

        double lateralOffset = (1 - columnIndex) * SubsystemsConfig.Limelight.Maze.COLUMN_SPACING_INCHES;

        return new Pose(
                SubsystemsConfig.Limelight.Maze.ORIGIN_X_INCHES
                        + scanForwardOffset * forwardX
                        + lateralOffset * leftX,
                SubsystemsConfig.Limelight.Maze.ORIGIN_Y_INCHES
                        + scanForwardOffset * forwardY
                        + lateralOffset * leftY,
                heading
        );
    }

    public static Pose getMazeExitPose(int rowIndex, int columnIndex) {
        validateMazeRowIndex(rowIndex);
        validateMazeColumnIndex(columnIndex);

        double heading = Math.toRadians(SubsystemsConfig.Limelight.Maze.HEADING_DEGREES);
        double forwardX = Math.cos(heading);
        double forwardY = Math.sin(heading);
        double leftX = -Math.sin(heading);
        double leftY = Math.cos(heading);
        double exitForwardOffset = rowIndex * SubsystemsConfig.Limelight.Maze.ROW_SPACING_INCHES
                + SubsystemsConfig.RobotDimensions.LENGTH / 2.0
                + SubsystemsConfig.Limelight.Maze.PASSED_ROW_REAR_CLEARANCE_INCHES;
        double lateralOffset = (1 - columnIndex) * SubsystemsConfig.Limelight.Maze.COLUMN_SPACING_INCHES;

        return new Pose(
                SubsystemsConfig.Limelight.Maze.ORIGIN_X_INCHES
                        + exitForwardOffset * forwardX
                        + lateralOffset * leftX,
                SubsystemsConfig.Limelight.Maze.ORIGIN_Y_INCHES
                        + exitForwardOffset * forwardY
                        + lateralOffset * leftY,
                heading
        );
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

    private MazeRowScan buildMazeRowScan(int rowIndex, List<LLResultTypes.ColorResult> targets, Pose robotPose) {
        BallColor color = getMazeRowColor(rowIndex);
        boolean[] occupiedColumns = new boolean[SubsystemsConfig.Limelight.Maze.COLUMN_COUNT];
        double[] columnAreas = new double[SubsystemsConfig.Limelight.Maze.COLUMN_COUNT];
        double[] columnXDegrees = new double[SubsystemsConfig.Limelight.Maze.COLUMN_COUNT];
        int acceptedTargetCount = 0;

        for (int columnIndex = 0; columnIndex < columnXDegrees.length; columnIndex++) {
            columnXDegrees[columnIndex] = Double.NaN;
        }

        if (targets != null) {
            for (LLResultTypes.ColorResult target : targets) {
                double area = target.getTargetArea();
                if (area < SubsystemsConfig.Limelight.Maze.MIN_TARGET_AREA) {
                    continue;
                }

                int columnIndex = getNearestMazeColumnIndex(rowIndex, target.getTargetXDegrees(), robotPose);
                if (columnIndex == -1) {
                    continue;
                }

                acceptedTargetCount++;
                if (area > columnAreas[columnIndex]) {
                    occupiedColumns[columnIndex] = true;
                    columnAreas[columnIndex] = area;
                    columnXDegrees[columnIndex] = target.getTargetXDegrees();
                }
            }
        }

        return new MazeRowScan(
                rowIndex,
                color,
                occupiedColumns,
                columnAreas,
                columnXDegrees,
                acceptedTargetCount
        );
    }

    private int getNearestMazeColumnIndex(int rowIndex, double xDegrees, Pose robotPose) {
        double bestError = Double.MAX_VALUE;
        int bestColumnIndex = -1;

        for (int columnIndex = 0; columnIndex < SubsystemsConfig.Limelight.Maze.COLUMN_COUNT; columnIndex++) {
            double expectedXDegrees = robotPose == null
                    || !SubsystemsConfig.Limelight.Maze.USE_POSE_BASED_COLUMN_TX
                    ? SubsystemsConfig.Limelight.Maze.COLUMN_X_DEGREES[columnIndex]
                    : getExpectedMazeColumnXDegrees(rowIndex, columnIndex, robotPose);
            double error = angleErrorDegrees(xDegrees, expectedXDegrees);
            if (error < bestError) {
                bestError = error;
                bestColumnIndex = columnIndex;
            }
        }

        return bestError <= SubsystemsConfig.Limelight.Maze.COLUMN_X_TOLERANCE_DEGREES
                ? bestColumnIndex
                : -1;
    }

    private double getExpectedMazeColumnXDegrees(int rowIndex, int columnIndex, Pose robotPose) {
        Pose cellPose = getMazeCellPose(rowIndex, columnIndex);
        double fieldBearing = Math.atan2(
                cellPose.getY() - robotPose.getY(),
                cellPose.getX() - robotPose.getX()
        );
        double relativeBearing = angleWrapRadians(fieldBearing - robotPose.getHeading());

        return SubsystemsConfig.Limelight.Maze.FIELD_BEARING_TO_TX_SIGN
                * Math.toDegrees(relativeBearing);
    }

    private double angleErrorDegrees(double actualDegrees, double expectedDegrees) {
        double error = actualDegrees - expectedDegrees;
        return Math.abs(Math.toDegrees(angleWrapRadians(Math.toRadians(error))));
    }

    private double angleWrapRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static void validateMazeRowIndex(int rowIndex) {
        if (rowIndex < 0 || rowIndex >= SubsystemsConfig.Limelight.Maze.ROW_COUNT) {
            throw new IllegalArgumentException("Maze row index out of range: " + rowIndex);
        }
    }

    private static void validateMazeColumnIndex(int columnIndex) {
        if (columnIndex < 0 || columnIndex >= SubsystemsConfig.Limelight.Maze.COLUMN_COUNT) {
            throw new IllegalArgumentException("Maze column index out of range: " + columnIndex);
        }
    }
}
