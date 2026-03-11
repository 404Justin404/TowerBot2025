package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight Subsystem for detecting purple game pieces in FTC DECODE
 * Pipeline 0: Purple ball detection
 */
public class LimeLight {

    private Limelight3A limelight;

    // Pipeline constant
    public static final int PURPLE_PIPELINE = 0;

    // Target detection data
    private boolean hasTarget = false;
    private double targetX = 0.0;      // Horizontal offset in degrees
    private double targetY = 0.0;      // Vertical offset in degrees
    private double targetArea = 0.0;   // Area percentage
    private int numTargets = 0;

//INIT
    public LimeLight(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(PURPLE_PIPELINE);
        limelight.start();
    }



//Update target data
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasTarget = true;
            targetX = result.getTx();
            targetY = result.getTy();
            targetArea = result.getTa();

            // Count
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            numTargets = colorResults.size();
        }
        else {
            hasTarget = false;
            targetX = 0.0;
            targetY = 0.0;
            targetArea = 0.0;
            numTargets = 0;
        }
    }


//chexck
    public boolean hasTarget() {
        return hasTarget;
    }


    /**
     * Get horizontal offset to target in degrees
     * Negative = target is left of crosshair
     * Positive = target is right of crosshair
     * @return tx value in degrees
     */
    public double getTargetX() {
        return targetX;
    }

    /**
     * Get vertical offset to target in degrees
     * Negative = target is below crosshair
     * Positive = target is above crosshair
     * @return ty value in degrees
     */
    public double getTargetY() {
        return targetY;
    }

    /**
     * Get target area as percentage of screen
     * Useful for distance estimation (larger = closer)
     * @return area percentage (0-100)
     */
    public double getTargetArea() {
        return targetArea;
    }

    /**
     * Get number of detected targets
     * @return count of individual balls detected
     */
    public int getNumTargets() {
        return numTargets;
    }

    /**
     * Estimate distance to target based on area
     * @return estimated distance in inches (requires calibration)
     */
    public double estimateDistance() {
        if (!hasTarget || targetArea <= 0) {
            return -1; // Invalid
        }

        // Basic inverse relationship: distance ∝ 1/√area
        // CALIBRATE!!!
        final double CALIBRATION_CONSTANT = 100.0; // Adjust
        return CALIBRATION_CONSTANT / Math.sqrt(targetArea);
    }

    /**
     * Check if target is centered (within tolerance)
     * @param xTolerance horizontal tolerance in degrees (e.g., 2.0)
     * @param yTolerance vertical tolerance in degrees (e.g., 2.0)
     * @return true if target is centered within tolerance
     */
    public boolean isTargetCentered(double xTolerance, double yTolerance) {
        return hasTarget &&
                Math.abs(targetX) < xTolerance &&
                Math.abs(targetY) < yTolerance;
    }

//enable purple
    public void switchToPurple() {
        limelight.pipelineSwitch(PURPLE_PIPELINE);
    }

//list of color objects
    public List<LLResultTypes.ColorResult> getAllTargets() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getColorResults();
        }
        return java.util.Collections.emptyList();
    }

    /**
     * Get Limelight status information
     * @return LLStatus object with temp, CPU, FPS, pipeline info
     */
    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    /**
     * Get raw LLResult for advanced usage
     * @return Latest LLResult, or null if not available
     */
    public LLResult getRawResult() {
        return limelight.getLatestResult();
    }

    /**
     * Stop the Limelight (call in OpMode stop())
     */
    public void stop() {
        limelight.stop();
    }


    public String getTelemetryString() {
        if (hasTarget) {
            return String.format("Targets: %d | X: %.2f° | Y: %.2f° | Area: %.2f%%",
                    numTargets, targetX, targetY, targetArea);
        } else {
            return "No targets detected";
        }
    }
}