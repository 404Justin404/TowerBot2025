package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@TeleOp
public class PedroLocalizationTest extends LinearOpMode {

    public static Follower follower;
    List<LynxModule> lynxModules;

    @Override
    public void runOpMode()
    {
        follower = Constants.createFollower(this.hardwareMap);
        TelemetryManager panels;
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SmartLocalizerPedro localizer = (SmartLocalizerPedro) follower.getPoseTracker().getLocalizer();

        follower.setStartingPose(new Pose(72,72, Math.toRadians(180)));
        follower.startTeleopDrive();
        follower.update();
        waitForStart();

        lynxModules = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        while(opModeIsActive())
        {
            for (LynxModule lynxModule : lynxModules) {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            localizer.getTelemetry(telemetry);
            telemetry.update();
        }
    }
}
