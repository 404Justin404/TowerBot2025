package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.TargetApi;
import android.os.Build;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
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
    /** The name of the Left Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public String perpendicularEncoder = "frontLeft";

    /** The name of the Right Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public String parallelEncoder = "frontRight";

    public double mmPerTick = 1/19.85017497812804;

    /** The name of the Gyro in the hardware map(name of the analog port it is plugged into)
     * Default Value: "canandgyro" */
    public String gyroName = "canandgyro";


    /** The Pinpoint Refresh time / Frequency
     * Default Value: 1000*/
    public double pinpointTimeDelta = 1000;

    /** The Pinpoint Rejection Threshold
     * Default Value: 4*/
    public double pinpointRejectionThreshold = 4;

    public DcMotorSimple.Direction forwardEncoderDirection = DcMotorSimple.Direction.REVERSE;

    public  DcMotorSimple.Direction strafeEncoderDirection = DcMotorSimple.Direction.REVERSE;

    public PinpointConstants pinpointConstants = new PinpointConstants();

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

    public SmartLocalizerConstants gyroName(String gyroName) {
        this.gyroName = gyroName;
        return this;
    }

    public SmartLocalizerConstants pinpointConstants(PinpointConstants pinpointConstants)
    {
        this.pinpointConstants = pinpointConstants;
        return this;
    }

    public SmartLocalizerConstants encoderResolution(double encoderResolution)
    {
        this.mmPerTick = encoderResolution;
        return this;
    }

    public void defaults() {
        perpendicularEncoder = "leftFront";
        parallelEncoder = "rightRear";
        mmPerTick = 1/19.85017497812804;
        forwardEncoderDirection = DcMotorSimple.Direction.REVERSE;
        strafeEncoderDirection = DcMotorSimple.Direction.REVERSE;
        gyroName = "canandgyro";

        pinpointConstants = new PinpointConstants();
        pinpointTimeDelta = 1000;
        pinpointRejectionThreshold = 4;
    }
}
