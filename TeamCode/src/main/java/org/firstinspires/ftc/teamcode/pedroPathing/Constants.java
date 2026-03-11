package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5) //in kg
            .forwardZeroPowerAcceleration(-30.76761359961186)
            .lateralZeroPowerAcceleration(-59.17310439403229)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.025,
                    0,
                    0.00001,
                    0.6,
                    0.01
            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.09,
                    0,
                    0.02,
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
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,
                    0,
                    0.01,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    3,
                    0,
                    0.08,
                    0.01
            ));

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
        .xVelocity(86.55613984836369)
        .yVelocity(67.04370549720103)
        .useVoltageCompensation(true);

    public static SmartLocalizerConstants smartLocalizerConstants = new SmartLocalizerConstants()
            .perpendicularEncoder_HardwareMapName("frontRight")
            .parallelEncoder_HardwareMapName("frontLeft")
            .gyroName("canandgyro")
            .forwardTicksToInches(0.001989436789)
            .forwardPodY(51.89862/25.4)
            .strafeTicksToInches(0.001989436789)
            .strafePodX(0.74927323/25.4)
            .distanceUnit(DistanceUnit.MM)

            .pinpoint_HardwareMapName("pinpoint")
            .pinpointTimeDelta(1000)
            .pinpointRejectionThreshold(4)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

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
        return new OracleFollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .smartLocalizer(smartLocalizerConstants)
                .build();
    }
}
