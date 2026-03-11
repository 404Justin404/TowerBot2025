package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the SmartLocalizerConstants class. It holds many constants and parameters for our custom Localizer.
 * @author Victor Padurescu - 19071 SmartCluster
 * @version 1.0, 24.02.2025
 */

@TargetApi(Build.VERSION_CODES.N)
public class SmartLocalizerConstants {

    public double forwardPodY = 1;
    public double strafePodX = -2.5;

    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    public String pinpointHardwareMap = "pinpoint";

    public GoBildaPinpointDriver.EncoderDirection pinpoint_forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public GoBildaPinpointDriver.EncoderDirection pinpoint_strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public String perpendicularEncoder = "frontLeft";

    public String parallelEncoder = "frontRight";

    public double encoderResolution = 1/19.85017497812804;

    public String gyroName = "canandgyro";

    public double pinpointTimeDelta = 1000;

    public double pinpointRejectionThreshold = 4;

    public DcMotorSimple.Direction forwardEncoderDirection = DcMotorSimple.Direction.REVERSE;

    public DcMotorSimple.Direction strafeEncoderDirection = DcMotorSimple.Direction.REVERSE;
    /**
     * This creates a new SmartLocalizerConstants with default values.
     */
    public SmartLocalizerConstants() {
        defaults();
    }

    public SmartLocalizerConstants perpendicularEncoder_HardwareMapName(String perpendicularEncoder_HardwareMapName) {
        this.perpendicularEncoder = perpendicularEncoder_HardwareMapName;
        return this;
    }

    public SmartLocalizerConstants parallelEncoder_HardwareMapName(String parallelEncoder_HardwareMapName) {
        this.parallelEncoder = parallelEncoder_HardwareMapName;
        return this;
    }

    public SmartLocalizerConstants pinpointTimeDelta(double pinpointTimeDelta) {
        this.pinpointTimeDelta = pinpointTimeDelta;
        return this;
    }

    public SmartLocalizerConstants pinpointRejectionThreshold(double pinpointRejectionThreshold) {
        this.pinpointRejectionThreshold = pinpointRejectionThreshold;
        return this;
    }

    public SmartLocalizerConstants forwardEncoderDirection(DcMotorSimple.Direction forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public SmartLocalizerConstants strafeEncoderDirection(DcMotorSimple.Direction strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public SmartLocalizerConstants pinpointHardwareMap(String pinpoint)
    {
        this.pinpointHardwareMap = pinpoint;
        return this;
    }

    public SmartLocalizerConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public SmartLocalizerConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public SmartLocalizerConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public SmartLocalizerConstants gyroName(String gyroName) {
        this.gyroName = gyroName;
        return this;
    }

    public SmartLocalizerConstants encoderResolution(double encoderResolution)
    {
        this.encoderResolution = encoderResolution;
        return this;
    }

    public SmartLocalizerConstants pinpoint_forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection pinpoint_forwardEncoderDirection)
    {
        this.pinpoint_forwardEncoderDirection = pinpoint_forwardEncoderDirection;
        return this;
    }

    public SmartLocalizerConstants pinpoint_strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection pinpoint_strafeEncoderDirection)
    {
        this.pinpoint_strafeEncoderDirection = pinpoint_strafeEncoderDirection;
        return this;
    }

    public void defaults() {
        perpendicularEncoder = "leftFront";
        parallelEncoder = "rightRear";
        encoderResolution = 1/19.85017497812804;
        forwardEncoderDirection = DcMotorSimple.Direction.REVERSE;
        strafeEncoderDirection = DcMotorSimple.Direction.REVERSE;
        forwardPodY = 51.89862;
        strafePodX = 0.74927323;
        distanceUnit = DistanceUnit.MM;
        pinpoint_forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        pinpoint_strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        pinpointHardwareMap = "pinpoint";
        gyroName = "canandgyro";
        pinpointTimeDelta = 1000;
        pinpointRejectionThreshold = 4;
    }
}
