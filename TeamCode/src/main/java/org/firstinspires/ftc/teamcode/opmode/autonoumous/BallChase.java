package org.firstinspires.ftc.teamcode.opmode.autonoumous;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LimelightBallDetector;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.State;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.calibration.new_fsm.Transition;

import java.util.List;



@Autonomous(name="Ball Chase", group="Auto")
public class BallChase extends LinearOpMode {

    private Follower follower;
    private final CommandScheduler scheduler = new CommandScheduler();
    private MecanumDrive drive;
    private LimelightBallDetector detector;

    private static final Pose startPose = new Pose(0, 0, Math.toRadians(-90));


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        detector = new LimelightBallDetector(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.4);
        follower.update();

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : lynxModules)
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        telemetry.addData("Status", "Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.activateAllPIDFs();
        follower.update();

        SequentialCommand auto = new SequentialCommand(
                Command.builder()
                        .update(() -> {
                            telemetry.addData("Waiting for ball", detector.getBallCount());
                            telemetry.update();
                        })
                        .finished(detector::hasBallsDetected)
                        .build(),

                detector.DriveToClosestBall(follower, detector, telemetry)
        );

        scheduler.schedule(auto);

        while (opModeIsActive()) {
            for (LynxModule m : lynxModules) {
                m.clearBulkCache();
                m.getBulkData();
            }

            scheduler.update();
            follower.update();

            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("Balls seen", detector.getBallCount());
            telemetry.update();
        }

        detector.stop();
    }
}