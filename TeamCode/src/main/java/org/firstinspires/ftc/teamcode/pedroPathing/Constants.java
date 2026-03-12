package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Configurable
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5) //in kg
            .forwardZeroPowerAcceleration(-21.104386178100388)
            .lateralZeroPowerAcceleration(-44.94161067829796)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.0009,
                    0.6,
                    0.001
            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.37,
                    0,
                    0.047,
                    0.007
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.09,
                    0.10
            ))

            //secondary PID Controllers, for finer control of the robot
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0035,
                    0,
                    0.0003,
                    0.6,
                    0.000
            ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.03,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0,
                    0.02,
                    0.01
            ))
            .translationalPIDFSwitch(1.5)
            .headingPIDFSwitch(0.1)
            .drivePIDFSwitch(20);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.9)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(75.32920881839135)
            .yVelocity(59.633986265083266)
            .useVoltageCompensation(true);

//    public static PinpointConstants pinpointConstants = new PinpointConstants()
//            .forwardPodY(51.89862/25.4) //offset of the forward encoder from the center of the robot in inches
//            .strafePodX(0.74927323/25.4) //offset of the strafe encoder from the center of the robot in inches
//            .distanceUnit(DistanceUnit.MM)
//            .hardwareMapName("pinpoint")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SmartLocalizerConstants smartLocalizerConstants = new SmartLocalizerConstants()
            .parallelEncoder_HardwareMapName("frontRight")
            .perpendicularEncoder_HardwareMapName("frontLeft")
            .gyroName("canandgyro")

            .forwardPodY(-51.89862) //offset of the forward encoder from the center of the robot in inches
            .strafePodX(0.74927323) //offset of the strafe encoder from the center of the robot in inches
            .distanceUnit(DistanceUnit.MM)
            .pinpointHardwareMap("pinpoint")

            .pinpoint_forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .pinpoint_strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)

            .encoderResolution(1/19.85017497812804) // ((Also overrides Pinpoint's resolution)) - R
            .pinpointTimeDelta(1000)
            .pinpointRejectionThreshold(3)
            .forwardEncoderDirection(DcMotorSimple.Direction.FORWARD)
            .strafeEncoderDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.17,
            0.13,
            0.007,
            100,
            1.1,
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
