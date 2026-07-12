package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.configurations.SubsystemsConfig;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .centripetalScaling(0)
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.1, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.08, 0.02))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.17, 0.059118688115056765, 0.0022617644311522123));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelIMULocalizer(new ThreeWheelIMUConstants()
                        .forwardTicksToInches(SubsystemsConfig.Localizer.FORWARD_TICKS_TO_INCHES)
                        .strafeTicksToInches(SubsystemsConfig.Localizer.STRAFE_TICKS_TO_INCHES)
                        .turnTicksToInches(SubsystemsConfig.Localizer.TURN_TICKS_TO_INCHES)
                        .leftPodY(SubsystemsConfig.Localizer.LEFT_POD_Y)
                        .rightPodY(SubsystemsConfig.Localizer.RIGHT_POD_Y)
                        .strafePodX(SubsystemsConfig.Localizer.STRAFE_POD_X)
                        .leftEncoder_HardwareMapName(SubsystemsConfig.Localizer.LEFT_ENCODER_NAME)
                        .rightEncoder_HardwareMapName(SubsystemsConfig.Localizer.RIGHT_ENCODER_NAME)
                        .strafeEncoder_HardwareMapName(SubsystemsConfig.Localizer.STRAFE_ENCODER_NAME)
                        .leftEncoderDirection((int) SubsystemsConfig.Localizer.LEFT_ENCODER_DIRECTION)
                        .rightEncoderDirection((int) SubsystemsConfig.Localizer.RIGHT_ENCODER_DIRECTION)
                        .strafeEncoderDirection((int) SubsystemsConfig.Localizer.STRAFE_ENCODER_DIRECTION)
                        .IMU_HardwareMapName(SubsystemsConfig.Localizer.IMU_HARDWARE_MAP_NAME)
                        .IMU_Orientation(SubsystemsConfig.Localizer.IMU_ORIENTATION)
                )
                .mecanumDrivetrain(new MecanumConstants()
                        .useBrakeModeInTeleOp(true)
                        .maxPower(SubsystemsConfig.Drivetrain.MAX_POWER)
                        .leftFrontMotorName(SubsystemsConfig.Drivetrain.FRONT_LEFT_MOTOR_NAME)
                        .leftRearMotorName(SubsystemsConfig.Drivetrain.BACK_LEFT_MOTOR_NAME)
                        .rightFrontMotorName(SubsystemsConfig.Drivetrain.FRONT_RIGHT_MOTOR_NAME)
                        .rightRearMotorName(SubsystemsConfig.Drivetrain.BACK_RIGHT_MOTOR_NAME)
                        .leftFrontMotorDirection(SubsystemsConfig.Drivetrain.FRONT_LEFT_MOTOR_DIRECTION)
                        .leftRearMotorDirection(SubsystemsConfig.Drivetrain.BACK_LEFT_MOTOR_DIRECTION)
                        .rightFrontMotorDirection(SubsystemsConfig.Drivetrain.FRONT_RIGHT_MOTOR_DIRECTION)
                        .rightRearMotorDirection(SubsystemsConfig.Drivetrain.BACK_RIGHT_MOTOR_DIRECTION)
                        .xVelocity(SubsystemsConfig.Drivetrain.X_VELOCITY)
                        .yVelocity(SubsystemsConfig.Drivetrain.Y_VELOCITY)
                )
                .build();
    }
}
