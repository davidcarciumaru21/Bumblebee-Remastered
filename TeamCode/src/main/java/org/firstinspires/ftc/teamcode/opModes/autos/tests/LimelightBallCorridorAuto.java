package org.firstinspires.ftc.teamcode.opModes.autos.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous(name = "Limelight Ball Corridor Auto", group = "Vision")
public class LimelightBallCorridorAuto extends OpMode {

    private static final double AUTO_TIME_MS = 8000.0;
    private static final double FORWARD_POWER = 0.28;
    private static final double SEARCH_FORWARD_POWER = 0.16;
    private static final double MAX_STRAFE_POWER = 0.34;
    private static final double MAX_ROTATION_POWER = 0.20;

    private static final double GREEN_LEFT_TARGET_X_DEG = -14.0;
    private static final double PURPLE_RIGHT_TARGET_X_DEG = 14.0;
    private static final double STRAFE_KP = 0.018;
    private static final double HEADING_KP = 0.85;
    private static final double TARGET_MAX_AGE_MS = 400.0;
    private static final double MIN_TARGET_AREA = 0.01;
    private static final double LOST_TARGET_STOP_MS = 1200.0;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;
    private Limelight limelight;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime lostTargetTimer = new ElapsedTime();

    private double startHeading;
    private boolean hasEverSeenTarget = false;
    private String steeringMode = "waiting";

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.FRONT_LEFT_MOTOR_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.BACK_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.FRONT_RIGHT_MOTOR_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.BACK_RIGHT_MOTOR_NAME);

        frontLeft.setDirection(SubsystemsConfig.Drivetrain.FRONT_LEFT_MOTOR_DIRECTION);
        backLeft.setDirection(SubsystemsConfig.Drivetrain.BACK_LEFT_MOTOR_DIRECTION);
        frontRight.setDirection(SubsystemsConfig.Drivetrain.FRONT_RIGHT_MOTOR_DIRECTION);
        backRight.setDirection(SubsystemsConfig.Drivetrain.BACK_RIGHT_MOTOR_DIRECTION);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, SubsystemsConfig.Localizer.IMU_HARDWARE_MAP_NAME);
        limelight = new Limelight(hardwareMap, SubsystemsConfig.Limelight.GREEN_BALL_PIPELINE);
        limelight.beginAlternatingBallTracking();

        telemetry.addLine("Limelight ball corridor ready");
        telemetry.addData("green", "left");
        telemetry.addData("purple", "right");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        limelight.updateAlternatingBallTracking();
        addBallTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        lostTargetTimer.reset();
        startHeading = getHeading();
        hasEverSeenTarget = false;
    }

    @Override
    public void loop() {
        limelight.updateAlternatingBallTracking();

        if (autoTimer.milliseconds() >= AUTO_TIME_MS) {
            stopDrive();
            requestOpModeStop();
            return;
        }

        Limelight.BallTarget green = getUsableTarget(Limelight.BallColor.GREEN);
        Limelight.BallTarget purple = getUsableTarget(Limelight.BallColor.PURPLE);

        double strafe = calculateStrafe(green, purple);
        boolean hasTarget = green != null || purple != null;
        double forward = calculateForwardPower(hasTarget);
        double rotation = calculateHeadingCorrection();

        drive(forward, strafe, rotation);

        telemetry.addLine("=== Limelight Ball Corridor ===");
        telemetry.addData("time ms", "%.0f / %.0f", autoTimer.milliseconds(), AUTO_TIME_MS);
        telemetry.addData("mode", steeringMode);
        telemetry.addData("forward", "%.2f", forward);
        telemetry.addData("strafe", "%.2f", strafe);
        telemetry.addData("rotation", "%.2f", rotation);
        addBallTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
        limelight.stop();
    }

    private Limelight.BallTarget getUsableTarget(Limelight.BallColor color) {
        Limelight.BallTarget target = limelight.getRecentBallTarget(color, TARGET_MAX_AGE_MS);
        if (target == null || target.getArea() < MIN_TARGET_AREA) {
            return null;
        }

        return target;
    }

    private double calculateStrafe(Limelight.BallTarget green, Limelight.BallTarget purple) {
        if (green != null || purple != null) {
            hasEverSeenTarget = true;
            lostTargetTimer.reset();
        }

        if (green != null && purple != null && green.getXDegrees() < purple.getXDegrees()) {
            steeringMode = "green + purple";
            double corridorCenterX = (green.getXDegrees() + purple.getXDegrees()) / 2.0;
            return Range.clip(corridorCenterX * STRAFE_KP, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
        }

        if (green != null && (purple == null || green.getArea() >= purple.getArea())) {
            steeringMode = "green only";
            double error = green.getXDegrees() - GREEN_LEFT_TARGET_X_DEG;
            return Range.clip(error * STRAFE_KP, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
        }

        if (purple != null) {
            steeringMode = "purple only";
            double error = purple.getXDegrees() - PURPLE_RIGHT_TARGET_X_DEG;
            return Range.clip(error * STRAFE_KP, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
        }

        steeringMode = "searching";
        return 0.0;
    }

    private double calculateForwardPower(boolean hasTarget) {
        if (hasTarget) {
            return FORWARD_POWER;
        }

        if (!hasEverSeenTarget || lostTargetTimer.milliseconds() < LOST_TARGET_STOP_MS) {
            return SEARCH_FORWARD_POWER;
        }

        return 0.0;
    }

    private double calculateHeadingCorrection() {
        double headingError = angleWrap(startHeading - getHeading());
        return Range.clip(headingError * HEADING_KP, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double angleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private void drive(double forward, double strafe, double rotation) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotation),
                1.0
        );

        frontLeft.setPower((forward + strafe + rotation) / denominator);
        frontRight.setPower((forward - strafe - rotation) / denominator);
        backLeft.setPower((forward - strafe + rotation) / denominator);
        backRight.setPower((forward + strafe - rotation) / denominator);
    }

    private void stopDrive() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    private void addBallTelemetry() {
        telemetry.addData("active pipeline", limelight.getActivePipelineIndex());
        addTargetTelemetry("green", limelight.getBallTarget(Limelight.BallColor.GREEN));
        addTargetTelemetry("purple", limelight.getBallTarget(Limelight.BallColor.PURPLE));
    }

    private void addTargetTelemetry(String label, Limelight.BallTarget target) {
        if (target == null) {
            telemetry.addData(label, "not seen");
            return;
        }

        telemetry.addData(
                label,
                "tx %.1f deg, area %.2f, age %.0f ms",
                target.getXDegrees(),
                target.getArea(),
                target.getAgeMs()
        );
    }
}
