package org.firstinspires.ftc.teamcode.util.calibration;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@TeleOp(group = "Calibration")
public class LeverCalibration extends LinearOpMode {

    private static final CommandScheduler scheduler = new CommandScheduler();
    public static double targetPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        Turret turret = new Turret(this);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", turret.getHoodAngle());

            turret.setLeverAngle(targetPosition);

            scheduler.update();
            telemetry.update();
        }
    }
}