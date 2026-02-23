package org.firstinspires.ftc.teamcode.myroadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer.SmartLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@TeleOp(group = "quickstart")
public class DeadWheelDirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);

        if (!(drive.localizer instanceof SmartLocalizer)) {
            telemetry.addData("ERROR", "SmartLocalizer not found!");
            telemetry.update();
            return;
        }

        SmartLocalizer localizer = (SmartLocalizer) drive.localizer;

        telemetry.addLine("Dead Wheel Direction Test");
        telemetry.addLine();
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Push robot FORWARD");
        telemetry.addLine("   → Parallel should INCREASE");
        telemetry.addLine();
        telemetry.addLine("2. Push robot to the LEFT");
        telemetry.addLine("   → Perpendicular should INCREASE");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        int lastParallel = 0;
        int lastPerpendicular = 0;

        while (opModeIsActive()) {
            // Get current encoder positions
            int parallel = localizer.parallelEncoder.getPositionAndVelocity().position;
            int perpendicular = localizer.perpendicularEncoder.getPositionAndVelocity().position;

            // Calculate changes
            int parallelChange = parallel - lastParallel;
            int perpendicularChange = perpendicular - lastPerpendicular;

            // Update last values
            lastParallel = parallel;
            lastPerpendicular = perpendicular;

            // Display
            telemetry.addLine("=== PARALLEL (Forward/Backward) ===");
            telemetry.addData("Position", parallel);
            telemetry.addData("Change", "%d %s",
                    parallelChange,
                    parallelChange > 0 ? "↑" : (parallelChange < 0 ? "↓" : "—"));
            telemetry.addLine();

            telemetry.addLine("=== PERPENDICULAR (Left/Right) ===");
            telemetry.addData("Position", perpendicular);
            telemetry.addData("Change", "%d %s",
                    perpendicularChange,
                    perpendicularChange > 0 ? "↑" : (perpendicularChange < 0 ? "↓" : "—"));
            telemetry.addLine();

            telemetry.addLine("EXPECTED BEHAVIOR:");
            telemetry.addLine("Push FORWARD → Parallel ↑");
            telemetry.addLine("Push LEFT → Perpendicular ↑");

            telemetry.update();

            sleep(50); // Small delay for readability
        }
    }
}