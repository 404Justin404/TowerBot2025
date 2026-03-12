package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Rotation2d;
import com.smartcluster.oracleftc.math.Rotation2dDual;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.Twist2dDual;
import com.smartcluster.oracleftc.math.Vector2dDual;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.LinkedList;
import java.util.Queue;

public class SmartLocalizerPedro implements Localizer {

    // Hardwares ----

    public final com.acmerobotics.roadrunner.ftc.Encoder parallelEncoder, perpendicularEncoder;
    private final AnalogInput canandgyro;
    public final GoBildaPinpointDriver pinpoint;

    // --------------
    private Pose2dDual<Time> pose = new Pose2dDual<Time>(
            new Vector2dDual<Time>(
                    new DualNum<>(0),
                    new DualNum<>(0)
            ),
            Rotation2dDual.exp(new DualNum<>(0))
    );

    public SmartLocalizerConstants constants;
    private final LowPassFilter headingVelFilter= new LowPassFilter(0.35);
    public enum TypeOfCheck
    {
        VELOCITY_BASED,
        DISTANCE_BASED
    };
    public TypeOfCheck typeOfCheck = TypeOfCheck.DISTANCE_BASED;

    // Variables to read before sleeping at 1 AM
    private double gyroVoltageOffset;
    private DualNum<Time> lastParallel, lastPerpendicular;
    private Rotation2dDual<Time> lastHeading = Rotation2dDual.constant(Rotation2d.exp(0),1);
    private final ElapsedTime deltaTime = new ElapsedTime();
    private final ElapsedTime pinpointTime = new ElapsedTime();
    private final IMURotationTracker tracker = new IMURotationTracker();
    private int validPosesCount = 0, invalidPosesCount = 0;
    // --------------

    public SmartLocalizerPedro(HardwareMap map, SmartLocalizerConstants constants)
    {
        this(map, constants, new Pose());
    }

    public SmartLocalizerPedro(HardwareMap map, SmartLocalizerConstants constants, Pose startPose) {
        canandgyro = map.get(AnalogInput.class, constants.gyroName);
        gyroVoltageOffset = canandgyro.getVoltage();

        parallelEncoder = new OverflowEncoder(new RawEncoder(map.get(DcMotorEx.class, constants.parallelEncoder)));
        parallelEncoder.setDirection(constants.forwardEncoderDirection);
        lastParallel = new DualNum<>(parallelEncoder.getPositionAndVelocity().position);

        perpendicularEncoder = new OverflowEncoder(new RawEncoder(map.get(DcMotorEx.class, constants.perpendicularEncoder)));
        perpendicularEncoder.setDirection(constants.strafeEncoderDirection);
        lastPerpendicular = new DualNum<>(perpendicularEncoder.getPositionAndVelocity().position);

        pinpoint = map.get(GoBildaPinpointDriver.class, constants.pinpointHardwareMap);
        pinpoint.setEncoderResolution(1 / constants.encoderResolution, DistanceUnit.MM);
        pinpoint.setEncoderDirections(constants.pinpoint_forwardEncoderDirection, constants.pinpoint_strafeEncoderDirection);
        pinpoint.setOffsets(-constants.forwardPodY, constants.strafePodX, constants.distanceUnit);

        resetPinpoint();
        setPose(startPose);
        this.constants = constants;
    }

    @Override
    public Pose getPose() {
        return new Pose(pose.position.value().x, pose.position.value().y, pose.heading.value().log());
    }

    @Override
    public Pose getVelocity() {
        return new Pose(pose.velocity().value().linearVel.x, pose.velocity().value().linearVel.y, pose.velocity().value().angVel);
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(pose.velocity().value().linearVel.x, pose.velocity().value().linearVel.y);
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart); // As if one setPose wasn't enough...
    }

    @Override
    public void setPose(Pose setPose) {
        Pose2d translatedPose = new Pose2d(setPose.getX(), setPose.getY(), setPose.getHeading());
        setPosition(translatedPose);
    }

    private void setPosition(Pose2d newPose) // IT WAS POSE2D ORACLEFTC THIS WHOLE TIME???
    {
        gyroVoltageOffset=canandgyro.getVoltage()-newPose.heading.log() * 3.3/360;
        pose = new Pose2dDual<Time>(
                new Vector2dDual<Time>(
                        new DualNum<>(newPose.position.x),
                        new DualNum<>(newPose.position.y)
                ),
                Rotation2dDual.exp(new DualNum<>(newPose.heading.log())));

        Pose2D translatedPose = new Pose2D(DistanceUnit.INCH, newPose.position.x, newPose.position.y, AngleUnit.RADIANS, newPose.heading.log());
        pinpoint.setPosition(translatedPose);
    }

    @Override
    public void update() {
        double canandgyroHeading = Math.toRadians(AngleUnit.normalizeDegrees((canandgyro.getVoltage()-gyroVoltageOffset) * 360.0 / 3.3));

        DualNum<Time> parallel = new DualNum<>(parallelEncoder.getPositionAndVelocity().position, parallelEncoder.getPositionAndVelocity().velocity);
        DualNum<Time> perpendicular = new DualNum<>(perpendicularEncoder.getPositionAndVelocity().position, perpendicularEncoder.getPositionAndVelocity().velocity);
        Rotation2d heading = Rotation2d.exp(canandgyroHeading);

        DualNum<Time> parallelDelta = parallel.minus(lastParallel);
        DualNum<Time> perpendicularDelta = perpendicular.minus(lastPerpendicular);
        double headingDifference = heading.minus(lastHeading.value());
        double headingVel = headingVelFilter.update(tracker.calculateAngularVelocity(canandgyroHeading, System.nanoTime()/1E9));
        Rotation2dDual<Time> headingDelta=Rotation2dDual.exp(new DualNum<>(headingDifference, headingVel));
        deltaTime.reset();

        Twist2dDual<Time> updateTwist = new Twist2dDual<>(
                new Vector2dDual<>(
                        parallelDelta.minus(headingDelta.log().times(constants.forwardPodY*(1/constants.encoderResolution))).times(constants.encoderResolution),
                        perpendicularDelta.minus(headingDelta.log().times(constants.strafePodX*(1/constants.encoderResolution))).times(constants.encoderResolution)
                ).div(25.4),
                headingDelta.log()
        );

        if(pinpointTime.milliseconds()>constants.pinpointTimeDelta)
        {
            pinpoint.update();
            Pose2dDual<Time> newPose = getPinpointPosition();

            if(isValidPose(newPose))
            {
                pose=newPose;
                validPosesCount++;
            }
            else invalidPosesCount++; // Aw, robot too good.

            pinpointTime.reset();
        }

        pose = new Pose2dDual<>(pose.value().plus(updateTwist.value()), pose.value().plus(updateTwist.value()).times(updateTwist.velocity()));

        lastHeading=Rotation2dDual.constant(heading,1);
        lastParallel=new DualNum<>(parallel.get(0));
        lastPerpendicular=new DualNum<>(perpendicular.get(0));
    }

    public void getTelemetry(Telemetry telemetry)
    {
//        telemetry.addData("rawGyroAngle", AngleUnit.normalizeDegrees((-canandgyro.getVoltage()) * 360.0 / 3.3));
//        telemetry.addData("offsetGyroAngle", AngleUnit.normalizeDegrees((canandgyro.getVoltage()-gyroVoltageOffset) * 360.0 / 3.3));
//        telemetry.addData("pinpointFrequency", pinpoint.getFrequency());
//
//        telemetry.addData("parallelEncoder", parallelEncoder.getPositionAndVelocity().position);
//        telemetry.addData("perpendicularEncoder", perpendicularEncoder.getPositionAndVelocity().position);
//        telemetry.addData("internalHeading", pose.heading.value().log());


//        telemetry.addData("x", pose.position.value().x);
//        telemetry.addData("y", pose.position.value().y);
//        telemetry.addData("heading", Math.toDegrees(pose.heading.value().log()));

        telemetry.addData("Good/Bad Pinpoint Poses", validPosesCount + "/" + invalidPosesCount);


//         Pinpoint debug
//        com.smartcluster.oracleftc.math.Pose2d pinpointPose = getPinpointPosition().value();
//
//        telemetry.addData("Pinpoint X", pinpointPose.position.x);
//        telemetry.addData("Pinpoint Y", pinpointPose.position.y);
//        telemetry.addData("Pinpoint HEADING", Math.toDegrees(pinpointPose.heading.log()));
//         --------------
    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return parallelEncoder.getPositionAndVelocity().position;
    }

    @Override
    public double getLateralMultiplier() {
        return perpendicularEncoder.getPositionAndVelocity().position;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    public void resetPinpoint()
    {
        pinpoint.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            RobotLog.logStackTrace(e);
        }
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    @Override
    public void setX(double x) {
        Localizer.super.setX(x);
    }

    @Override
    public void setY(double y) {
        Localizer.super.setY(y);
    }

    @Override
    public void setHeading(double heading) {
        Localizer.super.setHeading(heading);
    }


    // FUNDAMENTALLY FUNCTIONS
    public Pose2dDual<Time> getPinpointPosition()
    {
        return new Pose2dDual<Time>(
                new Vector2dDual<Time>(
                        new DualNum<>(pinpoint.getPosX(DistanceUnit.MM), pinpoint.getVelX(DistanceUnit.MM)),
                        new DualNum<>(pinpoint.getPosY(DistanceUnit.MM), pinpoint.getVelY(DistanceUnit.MM))
                ).div(25.4),
                Rotation2dDual.exp(new DualNum<>(pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)))
        );
    }

    boolean isValidPose(Pose2dDual<Time> newPose)
    {
        boolean isNull = Double.isNaN(newPose.heading.log().get(0)) ||
                Double.isNaN(newPose.position.x.get(0)) ||
                Double.isNaN(newPose.position.y.get(0));


        boolean validSituation = false;

        switch(typeOfCheck)
        {
            case DISTANCE_BASED:
                validSituation = pose.position.minus(newPose.position).sqrNorm().get(0) <= constants.pinpointRejectionThreshold * constants.pinpointRejectionThreshold;
                break;
//            case VELOCITY_BASED:
//                validSituation = pose.velocity().linearVel.sqrNorm().get(0) <= velocityPositionRejectionThreshold * velocityPositionRejectionThreshold;
//                validSituation &= Math.abs(pose.heading.velocity().get(0)) <= velocityHeadingRejectionThreshold;
//                break;
        }
        return !isNull && validSituation;
    }

    public class IMURotationTracker {
        private  class YawReading {
            double yawRadians;
            double timestamp;

            YawReading(double yawRadians, double timestamp) {
                this.yawRadians = yawRadians;
                this.timestamp = timestamp;
            }
        }

        private final Queue<IMURotationTracker.YawReading> buffer = new LinkedList<>();
        private static final double WINDOW_SIZE = 0.05; // 50 ms window in seconds

        public double calculateAngularVelocity(double currentYawDegrees, double currentTime) {
            // Convert yaw from degrees to radians
            double currentYaw = currentYawDegrees;

            // Update buffer with new reading
            buffer.add(new IMURotationTracker.YawReading(currentYaw, currentTime));

            // Remove outdated values (older than 50ms)
            while (buffer.size() > 1 && (currentTime - buffer.peek().timestamp) > WINDOW_SIZE) {
                buffer.poll();
            }

            // Compute angular velocity over the stored window
            if (buffer.size() > 1) {
                IMURotationTracker.YawReading oldest = buffer.peek();
                double totalDeltaYaw = wrapAngle(currentYaw - oldest.yawRadians);
                double deltaT = currentTime - oldest.timestamp;
                if (deltaT > 0) {
                    return totalDeltaYaw / deltaT; // Angular velocity in radians per second
                }
            }

            return 0.0; // Default when buffer is not full
        }

        private double wrapAngle(double angle) {
            // Ensures yaw difference is within [-π, π] range to handle wrap-around correctly
            if (angle > Math.PI) {
                return angle - 2 * Math.PI;
            } else if (angle < -Math.PI) {
                return angle + 2 * Math.PI;
            }
            return angle;
        }
    }

}
