package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5)
            .forwardZeroPowerAcceleration(-22.54897814291522)
            .lateralZeroPowerAcceleration(-61.636961122823166)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.025,
                    0,
                    0.00005,
                    0.6,
                    0.01
            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.025,
                    0.015
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.5,
                    0,
                    0,
                    0.1
            ))

            //secondary PID Controllers, for finer control of the robot

            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.06,
                    0,
                    0.0001,
                    0.6,
                    0.03

            ))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.065,
                    0,
                    0.01,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.08,
                    0.01
            ))
            ;//in kg

public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("frontRight")
        .rightRearMotorName("backRight")
        .leftRearMotorName("backLeft")
        .leftFrontMotorName("frontLeft")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(81.96620478592521)
        .yVelocity(61.04370549720103);

public static PinpointConstants pinpointConstants = new PinpointConstants()
        .forwardPodY(51.89862)//offset of the forward encoder from the center of the robot in inches
        .strafePodX(0.74927323)//offset of the strafe encoder from the center of the robot in inches
        .distanceUnit(DistanceUnit.MM)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.17,
            0.13,
            0.007,
            100,
            1.24,
            10,
            1.1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
