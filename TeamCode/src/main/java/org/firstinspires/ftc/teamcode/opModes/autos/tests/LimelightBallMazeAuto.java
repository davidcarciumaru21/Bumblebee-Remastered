package org.firstinspires.ftc.teamcode.opModes.autos.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.configurations.ShootingConfig;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.global.enums.managersEnums.ShootingManagerState;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Limelight Ball Maze Auto", group = "Vision")
public class LimelightBallMazeAuto extends OpMode {

    private enum State {
        PIPELINE_SETTLE,
        MOVING_TO_SCAN_COLUMN,
        SCANNING_COLUMN,
        MOVING_TO_OPEN_COLUMN,
        RETURNING_TO_START,
        RUNNING_LEARNED_PATH,
        RETURNING_TO_BASE,
        COLLECTING_MAZE_BALLS,
        COLLECTING_AT_BASE,
        STARTING_SHOT,
        SHOOTING_BALLS,
        STOPPING_SHOT,
        DONE,
        FAILED
    }

    private static final int CENTER_COLUMN = 1;
    private static final int[] SCAN_ORDER = {CENTER_COLUMN, 0, 2};
    private static final double ARRIVAL_TOLERANCE_INCHES = 2.0;
    private static final double STANDARD_SHOT_BALL_COUNT = 3.0;

    private Follower follower;
    private Limelight limelight;
    private VoltageSensor voltageSensor;
    private Flywheel flywheel;
    private Deflector deflector;
    private Turret turret;
    private Stopper stopper;
    private Intake intake;
    private Indexer indexer;
    private IntakingManager intakingManager;
    private ShootingManager shootingManager;

    private final Pose closeMidGoalPose = ShootingConfig.Goals.BLUE_GOAL_POSE;
    private final Pose farGoalPose = ShootingConfig.Goals.BLUE_FAR_GOAL_POSE;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final int[] openColumns = new int[SubsystemsConfig.Limelight.Maze.ROW_COUNT];
    private final boolean[] occupiedColumns = new boolean[SubsystemsConfig.Limelight.Maze.COLUMN_COUNT];
    private final double[] columnAreas = new double[SubsystemsConfig.Limelight.Maze.COLUMN_COUNT];

    private State state = State.PIPELINE_SETTLE;
    private int rowIndex = 0;
    private int scanOrderIndex = 0;
    private int scanColumnIndex = CENTER_COLUMN;
    private List<Pose> returnWaypoints = new ArrayList<>();
    private int returnWaypointIndex = 0;
    private List<Pose> learnedWaypoints = new ArrayList<>();
    private int learnedWaypointIndex = 0;
    private List<Pose> collectionWaypoints = new ArrayList<>();
    private List<Integer> collectionBallRows = new ArrayList<>();
    private List<Integer> collectionBallColumns = new ArrayList<>();
    private int collectionWaypointIndex = 0;
    private int plannedBallCount = 0;
    private int shotBallCount = 0;
    private boolean collectBallsAfterBaseReturn = false;
    private boolean finishAfterBaseReturn = false;
    private boolean shootAfterBaseReturn = false;
    private boolean resumeCollectionAfterShot = false;
    private Pose activeTargetPose;
    private String status = "waiting";

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.PATH_MAX_POWER);
        follower.update();

        limelight = new Limelight(hardwareMap, SubsystemsConfig.Limelight.GREEN_BALL_PIPELINE);

        voltageSensor = new VoltageSensor(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        deflector = new Deflector(hardwareMap);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intakingManager = new IntakingManager(intake, indexer);
        shootingManager = new ShootingManager(flywheel, deflector, turret, stopper, intakingManager);

        for (int index = 0; index < openColumns.length; index++) {
            openColumns[index] = -1;
        }

        prepareRow(0);
        addTelemetry();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        limelight.update();
        addTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        prepareRow(0);
    }

    @Override
    public void loop() {
        follower.update();
        limelight.update();
        voltageSensor.update();

        switch (state) {
            case PIPELINE_SETTLE:
                if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PIPELINE_SETTLE_MS) {
                    beginScanColumn(0);
                }
                break;

            case MOVING_TO_SCAN_COLUMN:
                if (isAtActiveTarget() || !follower.isBusy()) {
                    follower.breakFollowing();
                    startColumnScan();
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PATH_TIMEOUT_MS) {
                    fail("scan move timeout");
                }
                break;

            case SCANNING_COLUMN:
                recordCurrentColumnFrame();

                if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.COLUMN_SCAN_TIME_MS) {
                    if (scanOrderIndex + 1 < SCAN_ORDER.length) {
                        beginScanColumn(scanOrderIndex + 1);
                    } else {
                        driveToOpenColumn(decideOpenColumn());
                    }
                }
                break;

            case MOVING_TO_OPEN_COLUMN:
                if (isAtActiveTarget() || !follower.isBusy()) {
                    follower.breakFollowing();
                    int nextRow = rowIndex + 1;
                    if (nextRow >= SubsystemsConfig.Limelight.Maze.ROW_COUNT) {
                        follower.breakFollowing();
                        beginReturnToStart();
                    } else {
                        prepareRow(nextRow);
                    }
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PATH_TIMEOUT_MS) {
                    fail("open column timeout");
                }
                break;

            case RETURNING_TO_START:
                if (isAtActiveTarget() || !follower.isBusy()) {
                    follower.breakFollowing();
                    returnWaypointIndex++;
                    followNextReturnWaypoint();
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PATH_TIMEOUT_MS) {
                    fail("return timeout");
                }
                break;

            case RUNNING_LEARNED_PATH:
                if (isAtActiveTarget() || !follower.isBusy()) {
                    follower.breakFollowing();
                    learnedWaypointIndex++;
                    followNextLearnedWaypoint();
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PATH_TIMEOUT_MS) {
                    fail("learned path timeout");
                }
                break;

            case RETURNING_TO_BASE:
                if (collectBallsAfterBaseReturn || finishAfterBaseReturn) {
                    intakingManager.idle();
                } else {
                    intakingManager.pull();
                }

                if (isAtActiveTarget() || !follower.isBusy()) {
                    follower.breakFollowing();
                    returnWaypointIndex++;
                    followNextBaseReturnWaypoint();
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PATH_TIMEOUT_MS) {
                    fail("base return timeout");
                }
                break;

            case COLLECTING_MAZE_BALLS:
                intakingManager.pull();
                if (isAtActiveTarget() || !follower.isBusy()) {
                    follower.breakFollowing();
                    collectionWaypointIndex++;

                    if (isCollectedBallReadyToShoot()) {
                        beginReturnToBaseForCollectedBallShot();
                    } else {
                        followNextCollectionWaypoint();
                    }
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.PATH_TIMEOUT_MS) {
                    fail("ball collection timeout");
                }
                break;

            case COLLECTING_AT_BASE:
                intakingManager.pull();
                if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.BASE_COLLECTION_TIME_MS) {
                    beginShotStart();
                }
                break;

            case STARTING_SHOT:
                if (shootingManager.isShooting()) {
                    state = State.SHOOTING_BALLS;
                    status = "shooting balls";
                    stateTimer.reset();
                } else if (shootingManager.getState() == ShootingManagerState.IDLE
                        && turret.isAtPosition()
                        && flywheel.isAtSpeed()) {
                    shootingManager.shoot();
                    if (resumeCollectionAfterShot) {
                        status = "spinning up collected ball " + (shotBallCount + 1) + "/" + plannedBallCount;
                    } else {
                        status = "spinning up shooter";
                    }
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.SHOOT_START_TIMEOUT_MS) {
                    fail("shoot start timeout");
                }
                break;

            case SHOOTING_BALLS:
                if (stateTimer.milliseconds() >= getShotTimeMs()) {
                    shootingManager.stop();
                    state = State.STOPPING_SHOT;
                    status = "stopping shooter";
                    stateTimer.reset();
                }
                break;

            case STOPPING_SHOT:
                if (shootingManager.getState() == ShootingManagerState.IDLE) {
                    finishShotCycle("base shot complete");
                } else if (stateTimer.milliseconds() >= SubsystemsConfig.Limelight.Maze.SHOOT_STOP_TIMEOUT_MS) {
                    finishShotCycle("base shot stop timeout");
                }
                break;

            case DONE:
                break;

            case FAILED:
                follower.breakFollowing();
                break;
        }

        updateShootingTarget();
        addTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }

        if (limelight != null) {
            limelight.stop();
        }

        if (shootingManager != null) {
            shootingManager.stop();
        }

        if (intakingManager != null) {
            intakingManager.idle();
        }
    }

    private void prepareRow(int nextRowIndex) {
        rowIndex = nextRowIndex;
        scanOrderIndex = 0;
        scanColumnIndex = CENTER_COLUMN;

        for (int columnIndex = 0; columnIndex < occupiedColumns.length; columnIndex++) {
            occupiedColumns[columnIndex] = false;
            columnAreas[columnIndex] = 0.0;
        }

        limelight.setMazeRowPipeline(rowIndex);
        activeTargetPose = null;
        state = State.PIPELINE_SETTLE;
        status = "settling " + Limelight.getMazeRowColor(rowIndex) + " pipeline";
        stateTimer.reset();
    }

    private void beginScanColumn(int nextScanOrderIndex) {
        scanOrderIndex = nextScanOrderIndex;
        scanColumnIndex = SCAN_ORDER[scanOrderIndex];

        Pose scanPose = Limelight.getMazeScanPose(rowIndex, scanColumnIndex);
        activeTargetPose = scanPose;
        if (follower.getPose().distanceFrom(scanPose) <= ARRIVAL_TOLERANCE_INCHES) {
            startColumnScan();
            return;
        }

        PathChain path = buildPath(follower.getPose(), scanPose);
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.PATH_MAX_POWER);
        follower.followPath(path, false);

        state = State.MOVING_TO_SCAN_COLUMN;
        status = "moving to scan " + columnName(scanColumnIndex);
        stateTimer.reset();
    }

    private void startColumnScan() {
        state = State.SCANNING_COLUMN;
        status = "scanning " + columnName(scanColumnIndex);
        stateTimer.reset();
    }

    private void recordCurrentColumnFrame() {
        Limelight.BallTarget target = limelight.getLatestBallTarget();
        if (target == null || target.getArea() < SubsystemsConfig.Limelight.Maze.MIN_TARGET_AREA) {
            return;
        }

        double targetError = Math.abs(target.getXDegrees() - SubsystemsConfig.Limelight.Maze.SCAN_TARGET_X_DEGREES);
        if (targetError > SubsystemsConfig.Limelight.Maze.SCAN_TARGET_X_TOLERANCE_DEGREES) {
            return;
        }

        occupiedColumns[scanColumnIndex] = true;
        columnAreas[scanColumnIndex] = Math.max(columnAreas[scanColumnIndex], target.getArea());
    }

    private int decideOpenColumn() {
        int openColumnIndex = -1;

        for (int columnIndex = 0; columnIndex < occupiedColumns.length; columnIndex++) {
            if (!occupiedColumns[columnIndex]) {
                if (openColumnIndex != -1) {
                    return getBestGuessOpenColumn();
                }

                openColumnIndex = columnIndex;
            }
        }

        return openColumnIndex == -1 ? getBestGuessOpenColumn() : openColumnIndex;
    }

    private int getBestGuessOpenColumn() {
        int bestColumnIndex = SubsystemsConfig.Limelight.Maze.FALLBACK_COLUMN_INDEX;
        double bestArea = columnAreas[bestColumnIndex];
        double bestDistanceFromCenter = Math.abs(bestColumnIndex - CENTER_COLUMN);

        for (int columnIndex = 0; columnIndex < columnAreas.length; columnIndex++) {
            double area = columnAreas[columnIndex];
            double distanceFromCenter = Math.abs(columnIndex - CENTER_COLUMN);

            if (area < bestArea || (area == bestArea && distanceFromCenter < bestDistanceFromCenter)) {
                bestArea = area;
                bestDistanceFromCenter = distanceFromCenter;
                bestColumnIndex = columnIndex;
            }
        }

        return bestColumnIndex;
    }

    private void driveToOpenColumn(int openColumnIndex) {
        openColumns[rowIndex] = openColumnIndex;

        Pose openScanPose = Limelight.getMazeScanPose(rowIndex, openColumnIndex);
        Pose rowExitPose = Limelight.getMazeExitPose(rowIndex, openColumnIndex);
        activeTargetPose = rowExitPose;

        PathChain path = buildSafeOpenColumnPath(follower.getPose(), openScanPose, rowExitPose, rowExitPose);
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.PATH_MAX_POWER);
        follower.followPath(path, false);

        state = State.MOVING_TO_OPEN_COLUMN;
        status = "driving through " + columnName(openColumnIndex);
        stateTimer.reset();
    }

    private void beginReturnToStart() {
        returnWaypoints = buildReturnWaypoints();
        returnWaypointIndex = 0;
        status = "maze complete, returning";
        followNextReturnWaypoint();
    }

    private List<Pose> buildReturnWaypoints() {
        return buildReturnWaypoints(getStartPose());
    }

    private List<Pose> buildReturnWaypoints(Pose finalPose) {
        List<Pose> waypoints = new ArrayList<>();

        for (int memoryRow = SubsystemsConfig.Limelight.Maze.ROW_COUNT - 1; memoryRow >= 0; memoryRow--) {
            int openColumn = openColumns[memoryRow];
            if (!isKnownColumn(openColumn)) {
                openColumn = SubsystemsConfig.Limelight.Maze.FALLBACK_COLUMN_INDEX;
            }

            waypoints.add(Limelight.getMazeScanPose(memoryRow, openColumn));

            if (memoryRow > 0) {
                int previousOpenColumn = openColumns[memoryRow - 1];
                if (!isKnownColumn(previousOpenColumn)) {
                    previousOpenColumn = SubsystemsConfig.Limelight.Maze.FALLBACK_COLUMN_INDEX;
                }

                waypoints.add(Limelight.getMazeExitPose(memoryRow - 1, previousOpenColumn));
            }
        }

        waypoints.add(finalPose);
        return waypoints;
    }

    private List<Pose> buildReturnWaypointsFromRow(int currentRow, Pose finalPose) {
        List<Pose> waypoints = new ArrayList<>();

        for (int memoryRow = currentRow; memoryRow >= 0; memoryRow--) {
            int openColumn = openColumns[memoryRow];
            if (!isKnownColumn(openColumn)) {
                openColumn = SubsystemsConfig.Limelight.Maze.FALLBACK_COLUMN_INDEX;
            }

            waypoints.add(Limelight.getMazeExitPose(memoryRow, openColumn));
            waypoints.add(Limelight.getMazeScanPose(memoryRow, openColumn));
        }

        waypoints.add(finalPose);
        return waypoints;
    }

    private void followNextReturnWaypoint() {
        while (returnWaypointIndex < returnWaypoints.size()
                && follower.getPose().distanceFrom(returnWaypoints.get(returnWaypointIndex)) <= ARRIVAL_TOLERANCE_INCHES) {
            returnWaypointIndex++;
        }

        if (returnWaypointIndex >= returnWaypoints.size()) {
            activeTargetPose = getStartPose();
            beginLearnedSpeedRun();
            return;
        }

        activeTargetPose = returnWaypoints.get(returnWaypointIndex);
        PathChain path = buildPath(follower.getPose(), activeTargetPose);
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.PATH_MAX_POWER);
        follower.followPath(path, false);

        state = State.RETURNING_TO_START;
        status = "returning " + (returnWaypointIndex + 1) + "/" + returnWaypoints.size();
        stateTimer.reset();
    }

    private void beginLearnedSpeedRun() {
        learnedWaypoints = buildLearnedWaypoints();
        learnedWaypointIndex = 0;
        status = "returned, running learned path";
        followNextLearnedWaypoint();
    }

    private List<Pose> buildLearnedWaypoints() {
        List<Pose> waypoints = new ArrayList<>();

        for (int memoryRow = 0; memoryRow < SubsystemsConfig.Limelight.Maze.ROW_COUNT; memoryRow++) {
            int openColumn = openColumns[memoryRow];
            if (!isKnownColumn(openColumn)) {
                openColumn = SubsystemsConfig.Limelight.Maze.FALLBACK_COLUMN_INDEX;
            }

            waypoints.add(Limelight.getMazeScanPose(memoryRow, openColumn));
            waypoints.add(Limelight.getMazeExitPose(memoryRow, openColumn));
        }

        return waypoints;
    }

    private void followNextLearnedWaypoint() {
        while (learnedWaypointIndex < learnedWaypoints.size()
                && follower.getPose().distanceFrom(learnedWaypoints.get(learnedWaypointIndex)) <= ARRIVAL_TOLERANCE_INCHES) {
            learnedWaypointIndex++;
        }

        if (learnedWaypointIndex >= learnedWaypoints.size()) {
            beginReturnToBase(true);
            return;
        }

        activeTargetPose = learnedWaypoints.get(learnedWaypointIndex);
        PathChain path = buildPath(follower.getPose(), activeTargetPose);
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.LEARNED_PATH_MAX_POWER);
        follower.followPath(path, false);

        state = State.RUNNING_LEARNED_PATH;
        status = "learned run " + (learnedWaypointIndex + 1) + "/" + learnedWaypoints.size();
        stateTimer.reset();
    }

    private void beginReturnToBase(boolean startCollectionAfterReturn) {
        returnWaypoints = buildReturnWaypoints();
        returnWaypointIndex = 0;
        collectBallsAfterBaseReturn = startCollectionAfterReturn;
        finishAfterBaseReturn = false;
        shootAfterBaseReturn = false;

        if (startCollectionAfterReturn) {
            intakingManager.idle();
            status = "learned complete, returning before collection";
        } else {
            intakingManager.pull();
            status = "balls collected, returning to base";
        }

        followNextBaseReturnWaypoint();
    }

    private void beginReturnToBaseForCollectedBallShot() {
        returnWaypoints = buildReturnWaypointsFromRow(getLastCollectedBallRow(), getBaseShootPose());
        returnWaypointIndex = 0;
        collectBallsAfterBaseReturn = false;
        finishAfterBaseReturn = false;
        shootAfterBaseReturn = true;
        intakingManager.pull();
        status = "returning to base to shoot ball " + (shotBallCount + 1) + "/" + plannedBallCount;
        followNextBaseReturnWaypoint();
    }

    private void beginFinalParkingReturnToBase() {
        returnWaypoints = new ArrayList<>();
        returnWaypoints.add(getStartPose());
        returnWaypointIndex = 0;
        collectBallsAfterBaseReturn = false;
        finishAfterBaseReturn = true;
        shootAfterBaseReturn = false;
        intakingManager.idle();
        status = "all known balls shot, returning to base";
        followNextBaseReturnWaypoint();
    }

    private void followNextBaseReturnWaypoint() {
        while (returnWaypointIndex < returnWaypoints.size()
                && follower.getPose().distanceFrom(returnWaypoints.get(returnWaypointIndex)) <= ARRIVAL_TOLERANCE_INCHES) {
            returnWaypointIndex++;
        }

        if (returnWaypointIndex >= returnWaypoints.size()) {
            if (collectBallsAfterBaseReturn) {
                collectBallsAfterBaseReturn = false;
                beginMazeBallCollection();
            } else if (shootAfterBaseReturn) {
                shootAfterBaseReturn = false;
                beginCollectedBallShot();
            } else if (finishAfterBaseReturn) {
                finishAfterBaseReturn = false;
                intakingManager.idle();
                activeTargetPose = getStartPose();
                state = State.DONE;
                status = "all known balls shot";
            } else {
                beginBaseCollection();
            }
            return;
        }

        activeTargetPose = returnWaypoints.get(returnWaypointIndex);
        PathChain path = buildPath(follower.getPose(), activeTargetPose);
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.PATH_MAX_POWER);
        follower.followPath(path, false);

        state = State.RETURNING_TO_BASE;
        status = "returning to base " + (returnWaypointIndex + 1) + "/" + returnWaypoints.size();
        stateTimer.reset();
    }

    private void beginMazeBallCollection() {
        collectionWaypoints = buildBallCollectionWaypoints();
        collectionWaypointIndex = 0;
        shotBallCount = 0;
        resumeCollectionAfterShot = false;
        intakingManager.pull();

        if (collectionWaypoints.isEmpty()) {
            beginFinalParkingReturnToBase();
            return;
        }

        status = "collecting " + plannedBallCount + " known balls";
        followNextCollectionWaypoint();
    }

    private List<Pose> buildBallCollectionWaypoints() {
        List<Pose> waypoints = new ArrayList<>();
        collectionBallRows = new ArrayList<>();
        collectionBallColumns = new ArrayList<>();
        plannedBallCount = 0;
        int preferredColumn = CENTER_COLUMN;

        for (int memoryRow = 0; memoryRow < SubsystemsConfig.Limelight.Maze.ROW_COUNT; memoryRow++) {
            int openColumn = openColumns[memoryRow];
            if (!isKnownColumn(openColumn)) {
                openColumn = SubsystemsConfig.Limelight.Maze.FALLBACK_COLUMN_INDEX;
            }

            int firstBallColumn = getNearestBallColumn(openColumn, preferredColumn, -1);
            int secondBallColumn = getNearestBallColumn(openColumn, firstBallColumn, firstBallColumn);

            if (isKnownColumn(firstBallColumn)) {
                addBallCollectionPass(waypoints, memoryRow, firstBallColumn);
                preferredColumn = firstBallColumn;
            }

            if (isKnownColumn(secondBallColumn)) {
                addBallCollectionPass(waypoints, memoryRow, secondBallColumn);
                preferredColumn = secondBallColumn;
            }
        }

        return waypoints;
    }

    private int getNearestBallColumn(int openColumn, int preferredColumn, int excludedColumn) {
        int bestColumn = -1;
        int bestDistance = Integer.MAX_VALUE;

        for (int columnIndex = 0; columnIndex < SubsystemsConfig.Limelight.Maze.COLUMN_COUNT; columnIndex++) {
            if (columnIndex == openColumn || columnIndex == excludedColumn) {
                continue;
            }

            int distance = Math.abs(columnIndex - preferredColumn);
            if (bestColumn == -1 || distance < bestDistance) {
                bestColumn = columnIndex;
                bestDistance = distance;
            }
        }

        return bestColumn;
    }

    private void addBallCollectionPass(List<Pose> waypoints, int row, int column) {
        waypoints.add(Limelight.getMazeScanPose(row, column));
        waypoints.add(Limelight.getMazeExitPose(row, column));
        collectionBallRows.add(row);
        collectionBallColumns.add(column);
        plannedBallCount++;
    }

    private void followNextCollectionWaypoint() {
        while (collectionWaypointIndex < collectionWaypoints.size()
                && follower.getPose().distanceFrom(collectionWaypoints.get(collectionWaypointIndex)) <= ARRIVAL_TOLERANCE_INCHES) {
            collectionWaypointIndex++;
        }

        if (collectionWaypointIndex >= collectionWaypoints.size()) {
            beginFinalParkingReturnToBase();
            return;
        }

        activeTargetPose = collectionWaypoints.get(collectionWaypointIndex);
        PathChain path = buildPath(follower.getPose(), activeTargetPose);
        follower.setMaxPower(SubsystemsConfig.Limelight.Maze.PATH_MAX_POWER);
        follower.followPath(path, false);

        state = State.COLLECTING_MAZE_BALLS;
        status = "collecting balls " + (collectionWaypointIndex + 1) + "/" + collectionWaypoints.size();
        stateTimer.reset();
    }

    private boolean isCollectedBallReadyToShoot() {
        return collectionWaypointIndex > 0 && collectionWaypointIndex % 2 == 0;
    }

    private int getLastCollectedBallIndex() {
        return Math.max(0, collectionWaypointIndex / 2 - 1);
    }

    private int getLastCollectedBallRow() {
        int ballIndex = getLastCollectedBallIndex();
        if (ballIndex >= collectionBallRows.size()) {
            return 0;
        }

        return collectionBallRows.get(ballIndex);
    }

    private void beginCollectedBallShot() {
        intakingManager.idle();
        resumeCollectionAfterShot = true;
        activeTargetPose = getBaseShootPose();
        state = State.STARTING_SHOT;
        status = "base aiming ball " + (shotBallCount + 1) + "/" + plannedBallCount;
        stateTimer.reset();
    }

    private void beginBaseCollection() {
        activeTargetPose = getStartPose();
        intakingManager.pull();
        state = State.COLLECTING_AT_BASE;
        status = "collecting at base";
        stateTimer.reset();
    }

    private void beginShotStart() {
        intakingManager.idle();
        resumeCollectionAfterShot = false;
        state = State.STARTING_SHOT;
        status = "aiming shooter";
        stateTimer.reset();
    }

    private double getShotTimeMs() {
        return SubsystemsConfig.Flywheel.THREE_BALL_SHOT_TIME_MS / STANDARD_SHOT_BALL_COUNT;
    }

    private void finishShotCycle(String doneStatus) {
        intakingManager.idle();

        if (resumeCollectionAfterShot) {
            resumeCollectionAfterShot = false;
            shotBallCount++;

            if (collectionWaypointIndex >= collectionWaypoints.size()) {
                beginFinalParkingReturnToBase();
            } else {
                status = "shot collected ball " + shotBallCount + "/" + plannedBallCount;
                followNextCollectionWaypoint();
            }

            return;
        }

        activeTargetPose = getStartPose();
        state = State.DONE;
        status = doneStatus;
    }

    private boolean isKnownColumn(int columnIndex) {
        return columnIndex >= 0 && columnIndex < SubsystemsConfig.Limelight.Maze.COLUMN_COUNT;
    }

    private void fail(String reason) {
        follower.breakFollowing();
        shootingManager.stop();
        intakingManager.idle();
        state = State.FAILED;
        status = reason;
    }

    private boolean isAtActiveTarget() {
        return activeTargetPose != null
                && follower.getPose().distanceFrom(activeTargetPose) <= ARRIVAL_TOLERANCE_INCHES;
    }

    private Pose getStartPose() {
        return new Pose(
                SubsystemsConfig.Limelight.Maze.START_X_INCHES,
                SubsystemsConfig.Limelight.Maze.START_Y_INCHES,
                Math.toRadians(SubsystemsConfig.Limelight.Maze.START_HEADING_DEGREES)
        );
    }

    private Pose getBaseShootPose() {
        Pose startPose = getStartPose();
        Pose goalPose = getGoalPoseForPose(startPose);
        double headingToGoal = Math.atan2(
                goalPose.getY() - startPose.getY(),
                goalPose.getX() - startPose.getX()
        );

        return new Pose(startPose.getX(), startPose.getY(), headingToGoal);
    }

    private Pose getGoalPoseForPose(Pose currentPose) {
        return currentPose.distanceFrom(closeMidGoalPose) < ShootingConfig.Mid.MAX_DISTANCE
                ? closeMidGoalPose
                : farGoalPose;
    }

    private PathChain buildPath(Pose startPose, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(targetPose.getX(), targetPose.getY())
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
    }

    private PathChain buildPathVia(Pose startPose, Pose viaPose, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(viaPose.getX(), viaPose.getY())
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), viaPose.getHeading())
                .addPath(new BezierLine(
                        new Pose(viaPose.getX(), viaPose.getY()),
                        new Pose(targetPose.getX(), targetPose.getY())
                ))
                .setLinearHeadingInterpolation(viaPose.getHeading(), targetPose.getHeading())
                .build();
    }

    private PathChain buildPathVia(Pose startPose, Pose firstViaPose, Pose secondViaPose, Pose targetPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(firstViaPose.getX(), firstViaPose.getY())
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstViaPose.getHeading())
                .addPath(new BezierLine(
                        new Pose(firstViaPose.getX(), firstViaPose.getY()),
                        new Pose(secondViaPose.getX(), secondViaPose.getY())
                ))
                .setLinearHeadingInterpolation(firstViaPose.getHeading(), secondViaPose.getHeading())
                .addPath(new BezierLine(
                        new Pose(secondViaPose.getX(), secondViaPose.getY()),
                        new Pose(targetPose.getX(), targetPose.getY())
                ))
                .setLinearHeadingInterpolation(secondViaPose.getHeading(), targetPose.getHeading())
                .build();
    }

    private PathChain buildSafeOpenColumnPath(
            Pose startPose,
            Pose openScanPose,
            Pose rowExitPose,
            Pose targetPose
    ) {
        boolean atOpenScan = startPose.distanceFrom(openScanPose) <= ARRIVAL_TOLERANCE_INCHES;
        boolean exitIsTarget = rowExitPose.distanceFrom(targetPose) <= ARRIVAL_TOLERANCE_INCHES;

        if (atOpenScan && exitIsTarget) {
            return buildPath(startPose, targetPose);
        }

        if (atOpenScan) {
            return buildPathVia(startPose, rowExitPose, targetPose);
        }

        if (exitIsTarget) {
            return buildPathVia(startPose, openScanPose, rowExitPose);
        }

        return buildPathVia(startPose, openScanPose, rowExitPose, targetPose);
    }

    private void updateShootingTarget() {
        Pose currentPose = follower.getPose();
        Pose goalPose = getGoalPoseForPose(currentPose);

        double distance = currentPose.distanceFrom(goalPose)
                + SubsystemsConfig.RobotDimensions.TurreToCenterDistance;
        double globalAngleToGoal = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );

        double angleToGoal = globalAngleToGoal - currentPose.getHeading();
        angleToGoal = Math.atan2(Math.sin(angleToGoal), Math.cos(angleToGoal));

        Vector velocityVector = follower.poseTracker.getVelocity().copy();
        velocityVector.rotateVector(-currentPose.getHeading());
        shootingManager.update(distance, velocityVector, angleToGoal);
    }

    private void addTelemetry() {
        telemetry.addLine("=== Limelight Ball Maze ===");
        telemetry.addData("state", state);
        telemetry.addData("status", status);
        telemetry.addData("row", "%d / %d", rowIndex + 1, SubsystemsConfig.Limelight.Maze.ROW_COUNT);
        telemetry.addData("row color", Limelight.getMazeRowColor(Math.min(rowIndex, SubsystemsConfig.Limelight.Maze.ROW_COUNT - 1)));
        telemetry.addData("scan column", columnName(scanColumnIndex));
        telemetry.addData("return waypoint", "%d / %d", returnWaypointIndex, returnWaypoints.size());
        telemetry.addData("learned waypoint", "%d / %d", learnedWaypointIndex, learnedWaypoints.size());
        telemetry.addData("collection waypoint", "%d / %d", collectionWaypointIndex, collectionWaypoints.size());
        telemetry.addData("planned balls", plannedBallCount);
        telemetry.addData("shot balls", shotBallCount);
        telemetry.addData("active pipeline", limelight.getActivePipelineIndex());
        telemetry.addData("shooter", shootingManager.getState());
        telemetry.addData("intake", intakingManager.getState());
        telemetry.addData("turret ready", turret.isAtPosition());
        telemetry.addData(
                "turret",
                "angle %.1f target %.1f error %.1f",
                turret.getAngle(),
                turret.getAnglePosition(),
                turret.getAngleError()
        );
        telemetry.addData("flywheel", "%.0f / %.0f", flywheel.getRPM(), flywheel.getTargetRPM());

        Pose pose = follower.getPose();
        telemetry.addData("pose", "x %.1f, y %.1f, h %.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        if (activeTargetPose != null) {
            telemetry.addData(
                    "target pose",
                    "x %.1f, y %.1f, dist %.1f",
                    activeTargetPose.getX(),
                    activeTargetPose.getY(),
                    pose.distanceFrom(activeTargetPose)
            );
        }
        telemetry.addData("columns", formatColumns());
        telemetry.addData("areas", formatAreas());

        Limelight.BallTarget target = limelight.getLatestBallTarget();
        if (target == null) {
            telemetry.addData("target", "none");
        } else {
            telemetry.addData("target", "tx %.1f area %.3f", target.getXDegrees(), target.getArea());
        }

        telemetry.addData("maze path", formatOpenColumns());
    }

    private String formatColumns() {
        StringBuilder builder = new StringBuilder();

        for (int columnIndex = 0; columnIndex < SubsystemsConfig.Limelight.Maze.COLUMN_COUNT; columnIndex++) {
            if (columnIndex > 0) {
                builder.append(" ");
            }

            builder.append(columnName(columnIndex)).append(":");
            builder.append(occupiedColumns[columnIndex] ? "ball" : "open");
        }

        return builder.toString();
    }

    private String formatAreas() {
        StringBuilder builder = new StringBuilder();

        for (int columnIndex = 0; columnIndex < SubsystemsConfig.Limelight.Maze.COLUMN_COUNT; columnIndex++) {
            if (columnIndex > 0) {
                builder.append(" ");
            }

            builder.append(columnName(columnIndex)).append(":");
            builder.append(String.format("%.3f", columnAreas[columnIndex]));
        }

        return builder.toString();
    }

    private String formatOpenColumns() {
        StringBuilder builder = new StringBuilder();

        for (int index = 0; index < openColumns.length; index++) {
            if (index > 0) {
                builder.append(" | ");
            }

            builder.append("R").append(index + 1).append(":");
            builder.append(openColumns[index] == -1 ? "?" : columnName(openColumns[index]));
        }

        return builder.toString();
    }

    private String columnName(int columnIndex) {
        switch (columnIndex) {
            case 0:
                return "LEFT";
            case 1:
                return "CENTER";
            case 2:
                return "RIGHT";
            default:
                return "UNKNOWN";
        }
    }
}
