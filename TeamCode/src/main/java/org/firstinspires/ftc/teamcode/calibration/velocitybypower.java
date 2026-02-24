package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@Config
@TeleOp(group = "Calibration")
public class velocitybypower extends LinearOpMode {

    // ========== TUNABLE PARAMETERS ==========
    public static double targetVelocity = 0;
    public static double targetHood = 0;
    public static double intakePassivePower = 0.3; // Low speed for intake
    public static int telemetryUpdateMs = 100;

    // ========== INTERNAL STATE ==========
    private final CommandScheduler scheduler = new CommandScheduler();
    private Turret turret;
    private Intake intake;
    private List<LynxModule> lynxModules;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        initializeTelemetry();

        // Initialize hardware
        turret = new Turret(this);
        intake = new Intake(this);
        initializeLynxModules();

        // Schedule turret update command
        scheduler.schedule(turret.update());

        // Wait for start
        displayInitializationInfo();
        waitForStart();

        // Main loop
        runMainLoop();
    }

    private void initializeTelemetry() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );
        telemetry.setMsTransmissionInterval(telemetryUpdateMs);
    }

    private void initializeLynxModules() {
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void displayInitializationInfo() {
        telemetry.addLine("=== VELOCITY BY POWER CALIBRATION ===");
        telemetry.addLine("Use FTC Dashboard to adjust:");
        telemetry.addLine("  • targetVelocity (RPM)");
        telemetry.addLine("  • targetHood (position)");
        telemetry.addLine("  • intakePassivePower (0.0 - 1.0)");
        telemetry.addLine();
        telemetry.addLine("Intake will run continuously at low speed");
        telemetry.addLine("Ready to start!");
        telemetry.update();
    }

    private void runMainLoop() {
        while (opModeIsActive()) {
            // Clear bulk cache
            clearBulkCache();

            // Run intake at constant low speed
            intake.setPassivePower(intakePassivePower);

            // Get current velocity (call once and reuse)
            double currentVelocity = turret.getCurrentVelocity();

            // Update subsystem targets
            turret.setTargetVelocity(targetVelocity);
            turret.hoodact.setTarget(targetHood);
            turret.leveract.setTarget(0.15);

            // Update command scheduler
            scheduler.update();

            // Update telemetry
            updateTelemetry(currentVelocity);
        }
    }

    private void clearBulkCache() {
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
    }

    private void updateTelemetry(double currentVelocity) {
        telemetry.addLine("=== CURRENT STATUS ===");
        telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
        telemetry.addData("Current Velocity", "%.0f RPM", currentVelocity);
        telemetry.addData("Velocity Error", "%.0f RPM", targetVelocity - currentVelocity);
        telemetry.addLine();
        telemetry.addData("Target Hood", "%.2f", targetHood);
        telemetry.addData("Intake Power", "%.2f", intakePassivePower);
        telemetry.update();
    }
}