package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Flywheel Auto Calibration", group = "Calibration")
public class FlywheelCalibrationDashboard extends LinearOpMode {

    // Dashboard configurable parameters
    public static CalibrationMode MODE = CalibrationMode.AUTO_FULL;
    public static double TEST_VELOCITY = 3000; // Target velocity for testing
    public static double TEST_POWER = 0.5; // Manual power for testing

    // Auto calibration parameters
    public static double MIN_VELOCITY = 1000;
    public static double MAX_VELOCITY = 5000;
    public static int VELOCITY_STEPS = 8;
    public static double SETTLE_TIME_MS = 2000; // Time to wait for velocity to stabilize
    public static double DATA_COLLECTION_TIME_MS = 1000; // Time to collect data at each velocity

    // PID Tuning parameters (used after feedforward is calibrated)
    public static double KP_START = 0.0001;
    public static double KP_MAX = 0.01;
    public static double KI = 0.0;
    public static double KD = 0.0;

    // Calculated values (displayed on dashboard)
    public static double CALCULATED_KV = 0.0;
    public static double CALCULATED_KA = 0.0;
    public static double CALCULATED_KSTATIC = 0.0;
    public static double CALCULATED_KP = 0.0;

    // Motor configuration
    private DcMotorImplEx shooter1, shooter2;
    private FtcDashboard dashboard;
    private MultipleTelemetry telemetry;

    public enum CalibrationMode {
        AUTO_FULL,          // Full automatic calibration
        AUTO_FEEDFORWARD,   // Only calibrate feedforward
        AUTO_PID,          // Only tune PID (requires feedforward)
        MANUAL_TEST        // Manual testing mode
    }

    private static class VelocityDataPoint {
        double power;
        double velocity;

        VelocityDataPoint(double power, double velocity) {
            this.power = power;
            this.velocity = velocity;
        }
    }

    @Override
    public void runOpMode() {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());

        // Initialize motors
        shooter1 = hardwareMap.get(DcMotorImplEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter2 = hardwareMap.get(DcMotorImplEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Flywheel Calibration Ready");
        telemetry.addLine("Use FTC Dashboard to select calibration mode");
        telemetry.addLine("Modes:");
        telemetry.addLine("  AUTO_FULL: Complete calibration (recommended)");
        telemetry.addLine("  AUTO_FEEDFORWARD: Only calibrate feedforward");
        telemetry.addLine("  AUTO_PID: Only tune PID");
        telemetry.addLine("  MANUAL_TEST: Test with current values");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            switch (MODE) {
                case AUTO_FULL:
                    runFullCalibration();
                    break;
                case AUTO_FEEDFORWARD:
                    runFeedforwardCalibration();
                    break;
                case AUTO_PID:
                    runPIDTuning();
                    break;
                case MANUAL_TEST:
                    runManualTest();
                    break;
            }
        }

        // Stop motors
        setMotorPower(0);
    }

    private void runFullCalibration() {
        telemetry.addLine("=== FULL AUTO CALIBRATION ===");
        telemetry.update();
        sleep(1000);

        // Step 1: Find kStatic
        findKStatic();

        // Step 2: Calibrate feedforward (kV)
        runFeedforwardCalibration();

        // Step 3: Tune PID
        runPIDTuning();

        // Display final results
        displayFinalResults();
    }

    private void findKStatic() {
        telemetry.addLine("=== Finding kStatic (Static Friction) ===");
        telemetry.addLine("Slowly ramping power until motors start moving...");
        telemetry.update();

        double power = 0;
        double powerIncrement = 0.001;
        double velocityThreshold = 50; // RPM threshold to detect movement

        while (opModeIsActive() && power < 1.0) {
            setMotorPower(power);
            sleep(100);

            double velocity = getCurrentVelocity();

            telemetry.addData("Power", "%.4f", power);
            telemetry.addData("Velocity", "%.1f RPM", velocity);
            telemetry.update();

            if (velocity > velocityThreshold) {
                CALCULATED_KSTATIC = power;
                telemetry.addLine("✓ Movement detected!");
                telemetry.addData("kStatic", "%.4f", CALCULATED_KSTATIC);
                telemetry.update();
                sleep(2000);
                break;
            }

            power += powerIncrement;
        }

        setMotorPower(0);
        sleep(2000);
    }

    private void runFeedforwardCalibration() {
        telemetry.addLine("=== Feedforward Calibration ===");
        telemetry.addLine("Collecting velocity data at different power levels...");
        telemetry.update();
        sleep(1000);

        List<VelocityDataPoint> dataPoints = new ArrayList<>();

        // Collect data points across velocity range
        for (int i = 0; i < VELOCITY_STEPS; i++) {
            if (!opModeIsActive()) break;

            double targetPower = 0.3 + (0.6 * i / (VELOCITY_STEPS - 1)); // Range: 0.3 to 0.9

            telemetry.addLine("=================================");
            telemetry.addData("Step", "%d / %d", i + 1, VELOCITY_STEPS);
            telemetry.addData("Target Power", "%.3f", targetPower);
            telemetry.addLine("Waiting for velocity to stabilize...");
            telemetry.update();

            // Ramp up to target power
            setMotorPower(targetPower);
            sleep((long) SETTLE_TIME_MS);

            // Collect velocity samples
            List<Double> velocitySamples = new ArrayList<>();
            ElapsedTime timer = new ElapsedTime();

            while (opModeIsActive() && timer.milliseconds() < DATA_COLLECTION_TIME_MS) {
                double velocity = getCurrentVelocity();
                velocitySamples.add(velocity);

                telemetry.addLine("=================================");
                telemetry.addData("Step", "%d / %d", i + 1, VELOCITY_STEPS);
                telemetry.addData("Power", "%.3f", targetPower);
                telemetry.addData("Current Velocity", "%.1f RPM", velocity);
                telemetry.addData("Samples", "%d", velocitySamples.size());
                telemetry.update();

                sleep(50);
            }

            // Calculate average velocity
            double avgVelocity = velocitySamples.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(0);

            // Only add point if velocity is reasonable
            if (avgVelocity > 100) {
                dataPoints.add(new VelocityDataPoint(targetPower, avgVelocity));
                telemetry.addData("✓ Recorded", "Power: %.3f → Velocity: %.1f RPM",
                        targetPower, avgVelocity);
            } else {
                telemetry.addData("✗ Skipped", "Velocity too low: %.1f RPM", avgVelocity);
            }
            telemetry.update();
            sleep(500);
        }

        // Stop motors
        setMotorPower(0);
        sleep(1000);

        // Calculate kV using linear regression
        if (dataPoints.size() >= 3) {
            calculateKV(dataPoints);

            telemetry.addLine("=================================");
            telemetry.addLine("✓ Feedforward Calibration Complete!");
            telemetry.addData("kStatic", "%.6f", CALCULATED_KSTATIC);
            telemetry.addData("kV", "%.6f", CALCULATED_KV);
            telemetry.addLine("=================================");
            telemetry.update();
            sleep(3000);
        } else {
            telemetry.addLine("✗ Error: Not enough data points collected");
            telemetry.update();
            sleep(3000);
        }
    }

    private void calculateKV(List<VelocityDataPoint> dataPoints) {
        // Linear regression: power = kStatic + kV * velocity
        // Rearranged: (power - kStatic) = kV * velocity

        double sumVelocity = 0;
        double sumPower = 0;
        double sumVelocitySquared = 0;
        double sumPowerVelocity = 0;
        int n = dataPoints.size();

        for (VelocityDataPoint point : dataPoints) {
            sumVelocity += point.velocity;
            sumPower += point.power;
            sumVelocitySquared += point.velocity * point.velocity;
            sumPowerVelocity += point.power * point.velocity;
        }

        // Calculate kV using least squares regression
        double numerator = (n * sumPowerVelocity) - (sumVelocity * sumPower);
        double denominator = (n * sumVelocitySquared) - (sumVelocity * sumVelocity);

        if (denominator != 0) {
            CALCULATED_KV = numerator / denominator;

            // Recalculate kStatic using regression intercept
            double intercept = (sumPower - CALCULATED_KV * sumVelocity) / n;
            if (intercept > 0 && intercept < 0.3) {
                CALCULATED_KSTATIC = intercept;
            }
        }

        // Display all data points for verification
        telemetry.addLine("Data Points:");
        for (int i = 0; i < dataPoints.size(); i++) {
            VelocityDataPoint p = dataPoints.get(i);
            double predictedPower = CALCULATED_KSTATIC + CALCULATED_KV * p.velocity;
            double error = Math.abs(p.power - predictedPower);
            telemetry.addData(String.format("  Point %d", i + 1),
                    "Vel: %.0f RPM, Power: %.3f, Predicted: %.3f, Error: %.3f",
                    p.velocity, p.power, predictedPower, error);
        }
        telemetry.update();
    }

    private void runPIDTuning() {
        telemetry.addLine("=== PID Tuning ===");
        telemetry.addLine("Testing different kP values...");
        telemetry.update();
        sleep(1000);

        if (CALCULATED_KV == 0) {
            telemetry.addLine("✗ Error: Feedforward not calibrated yet!");
            telemetry.addLine("Run feedforward calibration first.");
            telemetry.update();
            sleep(3000);
            return;
        }

        // Test velocities
        double[] testVelocities = {2000, 3000, 4000, 3500};
        double bestKP = KP_START;
        double bestError = Double.MAX_VALUE;

        // Try different kP values
        for (double kP = KP_START; kP <= KP_MAX; kP *= 2) {
            if (!opModeIsActive()) break;

            telemetry.addLine("=================================");
            telemetry.addData("Testing kP", "%.6f", kP);
            telemetry.update();

            double totalError = 0;
            int validTests = 0;

            // Test at different velocities
            for (double targetVel : testVelocities) {
                if (!opModeIsActive()) break;

                double error = testVelocityControl(targetVel, kP);
                if (error >= 0) {
                    totalError += error;
                    validTests++;
                }
            }

            if (validTests > 0) {
                double avgError = totalError / validTests;

                telemetry.addData("Average Error", "%.1f RPM", avgError);
                telemetry.update();

                if (avgError < bestError) {
                    bestError = avgError;
                    bestKP = kP;
                }
            }

            setMotorPower(0);
            sleep(1000);
        }

        CALCULATED_KP = bestKP;

        telemetry.addLine("=================================");
        telemetry.addLine("✓ PID Tuning Complete!");
        telemetry.addData("Best kP", "%.6f", CALCULATED_KP);
        telemetry.addData("Average Error", "%.1f RPM", bestError);
        telemetry.addLine("=================================");
        telemetry.update();
        sleep(3000);
    }

    private double testVelocityControl(double targetVelocity, double kP) {
        telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        List<Double> errors = new ArrayList<>();
        double integral = 0;
        double lastError = 0;

        while (opModeIsActive() && timer.seconds() < 3.0) {
            double currentVelocity = getCurrentVelocity();
            double error = targetVelocity - currentVelocity;

            // Simple PID calculation
            integral += error * 0.02; // Assuming 50Hz update rate
            double derivative = (error - lastError) / 0.02;

            // Calculate control output
            double feedforward = CALCULATED_KSTATIC + CALCULATED_KV * targetVelocity;
            double feedback = kP * error + KI * integral + KD * derivative;
            double power = feedforward + feedback;

            // Clamp power
            power = Math.max(0, Math.min(1.0, power));

            setMotorPower(power);

            // Collect error data after settling
            if (timer.seconds() > 1.5) {
                errors.add(Math.abs(error));
            }

            telemetry.addData("Target", "%.0f RPM", targetVelocity);
            telemetry.addData("Current", "%.0f RPM", currentVelocity);
            telemetry.addData("Error", "%.0f RPM", error);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("FF", "%.3f", feedforward);
            telemetry.addData("FB", "%.3f", feedback);
            telemetry.update();

            lastError = error;
            sleep(20);
        }

        // Return average absolute error
        if (errors.isEmpty()) return -1;
        return errors.stream().mapToDouble(Double::doubleValue).average().orElse(-1);
    }

    private void runManualTest() {
        telemetry.addLine("=== Manual Test Mode ===");
        telemetry.addLine("Adjust TEST_VELOCITY or TEST_POWER in dashboard");
        telemetry.addLine("Press A for velocity control, release for power control");
        telemetry.addLine("Press STOP to exit");
        telemetry.update();

        boolean useVelocityControl = CALCULATED_KV > 0;

        double integral = 0;
        double lastError = 0;

        while (opModeIsActive()) {
            if (useVelocityControl && gamepad1.a) {
                // Velocity control mode
                double currentVelocity = getCurrentVelocity();
                double error = TEST_VELOCITY - currentVelocity;

                integral += error * 0.02;
                double derivative = (error - lastError) / 0.02;

                double feedforward = CALCULATED_KSTATIC + CALCULATED_KV * TEST_VELOCITY;
                double feedback = CALCULATED_KP * error + KI * integral + KD * derivative;
                double power = feedforward + feedback;

                power = Math.max(0, Math.min(1.0, power));
                setMotorPower(power);

                telemetry.addLine("Mode: VELOCITY CONTROL (A button)");
                telemetry.addData("Target", "%.0f RPM", TEST_VELOCITY);
                telemetry.addData("Current", "%.0f RPM", currentVelocity);
                telemetry.addData("Error", "%.0f RPM", error);
                telemetry.addData("Power", "%.3f", power);
                telemetry.addData("FF", "%.3f", feedforward);
                telemetry.addData("FB", "%.3f", feedback);

                lastError = error;
            } else {
                // Manual power mode
                setMotorPower(TEST_POWER);
                double currentVelocity = getCurrentVelocity();

                telemetry.addLine("Mode: MANUAL POWER");
                telemetry.addData("Power", "%.3f", TEST_POWER);
                telemetry.addData("Velocity", "%.0f RPM", currentVelocity);

                integral = 0;
                lastError = 0;
            }

            telemetry.addLine("---");
            telemetry.addLine("Current Calibration Values:");
            telemetry.addData("kStatic", "%.6f", CALCULATED_KSTATIC);
            telemetry.addData("kV", "%.6f", CALCULATED_KV);
            telemetry.addData("kP", "%.6f", CALCULATED_KP);
            telemetry.update();

            sleep(20);
        }
    }

    private void displayFinalResults() {
        telemetry.clear();
        telemetry.addLine("╔════════════════════════════════════╗");
        telemetry.addLine("║   CALIBRATION COMPLETE!            ║");
        telemetry.addLine("╚════════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("Copy these values to your Turret.java:");
        telemetry.addLine();
        telemetry.addLine("// Feedforward");
        telemetry.addData("kStatic", "%.6f", CALCULATED_KSTATIC);
        telemetry.addData("kV", "%.6f", CALCULATED_KV);
        telemetry.addData("kA", "%.6f", CALCULATED_KA);
        telemetry.addLine();
        telemetry.addLine("Code:");
        telemetry.addLine(String.format("MotorFeedforward(%.6f, %.6f, %.6f)",
                CALCULATED_KSTATIC, CALCULATED_KV, CALCULATED_KA));
        telemetry.addLine();
        telemetry.addLine("// PID");
        telemetry.addData("kP", "%.6f", CALCULATED_KP);
        telemetry.addData("kI", "%.6f", KI);
        telemetry.addData("kD", "%.6f", KD);
        telemetry.addLine();
        telemetry.addLine("Code:");
        telemetry.addLine(String.format("PIDController(%.6f, %.6f, %.6f)",
                CALCULATED_KP, KI, KD));
        telemetry.addLine();
        telemetry.addLine("════════════════════════════════════");
        telemetry.update();

        // Keep displaying until stop is pressed
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private void setMotorPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    private double getCurrentVelocity() {
        // Get velocity from shooter2 (using encoder ticks per revolution = 28 for motor)
        // Convert from ticks/sec to RPM
        double ticksPerSec = shooter2.getVelocity();
        double rps = ticksPerSec / 28.0; // 28 ticks per revolution
        double rpm = rps * 60.0;
        return rpm;
    }
}