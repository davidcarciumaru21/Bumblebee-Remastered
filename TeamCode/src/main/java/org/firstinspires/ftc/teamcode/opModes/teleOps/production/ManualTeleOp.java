package org.firstinspires.ftc.teamcode.opModes.teleOps.production;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;
import org.firstinspires.ftc.teamcode.subsystems.Deflector;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

/**
 * Manual TeleOp — controls all subsystems directly without managers or Pedro Pathing.
 * Field centric / robot centric toggle via share button.
 *
 * Controls:
 * Gamepad1:
 * - Left stick      — strafe / drive
 * - Right stick X   — rotate
 * - Share           — toggle field centric / robot centric
 * - Right trigger   — intake (held)
 * - Left trigger    — indexer pull (held)
 * - Right bumper    — deflector position +0.01
 * - Left bumper     — deflector position -0.01
 * - X               — stopper open
 * - Y               — stopper close
 *
 * Gamepad2:
 * - DPAD RIGHT      — turret +5°
 * - DPAD LEFT       — turret -5°
 * - DPAD UP         — flywheel speed +100 RPM
 * - DPAD DOWN       — flywheel speed -100 RPM
 */
@TeleOp(name = "Manual TeleOp", group = "production")
public class ManualTeleOp extends OpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU       imu;

    private VoltageSensor voltageSensor;
    private Intake        intake;
    private Indexer       indexer;
    private Deflector     deflector;
    private Stopper       stopper;
    private Turret        turret;
    private Flywheel      flywheel;

    private boolean fieldCentric   = true;
    private double  turretAngle    = 0.0;
    private double  flywheelSpeed  = 0.0;
    private double  deflectorPos   = SubsystemsConfig.Deflector.IDLE_POSITION;

    private static final double TURRET_STEP    = 5.0;
    private static final double FLYWHEEL_STEP  = 100.0;
    private static final double DEFLECTOR_STEP = 0.01;

    @Override
    public void init() {
        // drivetrain
        frontLeft  = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.FRONT_LEFT_MOTOR_NAME);
        backLeft   = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.BACK_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.FORNT_RIGHT_MOTOR_NAME);
        backRight  = hardwareMap.get(DcMotorEx.class, SubsystemsConfig.Drivetrain.BACK_RIGHT_MOTOR_NAME);

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

        imu = hardwareMap.get(IMU.class, "imu");

        // subsystems
        voltageSensor = new VoltageSensor(hardwareMap);
        intake        = new Intake(hardwareMap);
        indexer       = new Indexer(hardwareMap);
        deflector     = new Deflector(hardwareMap);
        stopper       = new Stopper(hardwareMap);
        turret        = new Turret(hardwareMap, voltageSensor);
        flywheel      = new Flywheel(hardwareMap, voltageSensor);
    }

    @Override
    public void loop() {

        // toggle field centric / robot centric
        if (gamepad1.shareWasPressed()) fieldCentric = !fieldCentric;

        // driving
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (fieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX    =  x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY    =  x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeft.setPower((rotY + rotX + rx)  / denominator);
            frontRight.setPower((rotY - rotX - rx) / denominator);
            backLeft.setPower((rotY - rotX + rx)   / denominator);
            backRight.setPower((rotY + rotX - rx)  / denominator);
        } else {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((y + x + rx)  / denominator);
            frontRight.setPower((y - x - rx) / denominator);
            backLeft.setPower((y - x + rx)   / denominator);
            backRight.setPower((y + x - rx)  / denominator);
        }

        // intake — held
        if (gamepad1.right_trigger > 0.1) intake.pull();
        else                              intake.idle();

        // indexer — held
        if (gamepad1.left_trigger > 0.1) indexer.pull();
        else                             indexer.idle();

        // deflector — increment/decrement position
        if (gamepad1.rightBumperWasPressed()) deflectorPos += DEFLECTOR_STEP;
        if (gamepad1.leftBumperWasPressed())  deflectorPos -= DEFLECTOR_STEP;
        deflectorPos = Math.max(0.0, Math.min(1.0, deflectorPos));
        deflector.setPosition(deflectorPos);

        // stopper
        if (gamepad1.xWasPressed()) stopper.open();
        if (gamepad1.yWasPressed()) stopper.close();

        // turret
        if (gamepad2.dpadRightWasPressed()) turretAngle += TURRET_STEP;
        if (gamepad2.dpadLeftWasPressed())  turretAngle -= TURRET_STEP;
        turret.setTargetAngle(turretAngle);

        // flywheel
        if (gamepad2.dpadUpWasPressed())   flywheelSpeed += FLYWHEEL_STEP;
        if (gamepad2.dpadDownWasPressed()) flywheelSpeed -= FLYWHEEL_STEP;
        flywheelSpeed = Math.max(0.0, flywheelSpeed);
        flywheel.setRPM(flywheelSpeed);

        // update subsystems
        voltageSensor.update();
        intake.update();
        indexer.update();
        deflector.update();
        stopper.update();
        turret.update();
        flywheel.update();

        // telemetry
        telemetry.addData("mode",           fieldCentric ? "field centric" : "robot centric");
        telemetry.addData("turret angle",   turretAngle);
        telemetry.addData("flywheel RPM",   flywheelSpeed);
        telemetry.addData("deflector pos",  deflectorPos);
        telemetry.addData("deflector",      deflector.getState());
        telemetry.addData("stopper",        stopper.getState());
        telemetry.addData("intake",         intake.getState());
        telemetry.addData("indexer",        indexer.getState());
        telemetry.addData("flywheel state", flywheel.getState());
        telemetry.addData("turret state",   turret.getState());
        telemetry.addData("voltage",        voltageSensor.getVoltage());
        telemetry.update();
    }
}