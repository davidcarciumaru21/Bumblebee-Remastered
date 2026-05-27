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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(-0.001966)
            .strafeTicksToInches(-0.001988)
            .turnTicksToInches(-0.001996)
            .leftPodY(4.25)
            .rightPodY(-4.25)
            .strafePodX(-2.93)
            .leftEncoder_HardwareMapName("BackRight")
            .rightEncoder_HardwareMapName("Indexer")
            .strafeEncoder_HardwareMapName("Intake")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) // IN CODU VECHI ASTA NU E
            .leftFrontMotorName("FrontLeft")
            .leftRearMotorName("BackLeft")
            .rightFrontMotorName("FrontRight")
            .rightRearMotorName("BackRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(79.7768)
            .yVelocity(64.4725);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .centripetalScaling(0)
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.1, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5,0,0.08,0.02))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.17, 0.059118688115056765, 0.0022617644311522123));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelIMULocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
