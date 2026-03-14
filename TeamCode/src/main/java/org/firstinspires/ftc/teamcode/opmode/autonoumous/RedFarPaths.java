package org.firstinspires.ftc.teamcode.opmode.autonoumous;

import android.media.audiofx.Visualizer;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@Configurable
@Autonomous(name="Red Far Auto", group="Auto")
public class RedFarPaths extends LinearOpMode {

    private static Command PedroToCommand(PathChain path, boolean holdEnd) {
        return Command.builder()
                .init(() ->
                {
                    follower.followPath(path, holdEnd);
                })
                .finished(() -> !follower.isBusy())
                .build();
    }
    List<LynxModule> lynxModules;


    private static Follower follower;
    private final CommandScheduler scheduler = new CommandScheduler();
    private final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(50);
    private TelemetryManager panelsTelemetry;
    private static SequentialCommand SequenceAuto;
    protected boolean isRed = false;
    private final Pose startPose = new Pose(120.0337197049526, 127.72181243414121, Math.toRadians(-143));
    private final Pose goalPose = new Pose(140.5,140.5,Math.toRadians(-135));
    private double shootPower = 2600;
    private double turretPitch = 0.1;

    public PathChain PreShoot;
    public PathChain Stack2;
    public PathChain Stack2Shoot;
    public PathChain GateOpen;
    public PathChain GateShoot;
    public PathChain Stack3;
    public PathChain Stack3Shoot;
    public PathChain HumanIntake;
    public PathChain HumanShoot;
    public PathChain Stack1;
    public PathChain Stack1Shoot;

    public PathChain Gate;
    public PathChain ReleaseGate;
    public PathChain ShootRelease;

    private Turret flywheel;
    private MecanumDrive drive;
    private Intake intake;

    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }


    public void buildPaths() {

        PreShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.034, 127.722),
                                new Pose(86.044, 85.619)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-137))
                .build();

        Stack3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.044, 85.619),
                                new Pose(98.865, 82.946),
                                new Pose(125.012, 82.798)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Stack3Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.012, 82.798),
                                new Pose(85.907, 85.570)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-137))
                .build();

        Stack2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.907, 85.570),
                                new Pose(88.609, 58.171),
                                new Pose(130.736, 59.037)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Stack2Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.736, 59.037),
                                new Pose(85.996, 85.089)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-137))
                .build();

        GateOpen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.996, 85.089),
                                new Pose(109.602, 46.525),
                                new Pose(130.711, 57.759)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        GateShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.711, 57.759),
                                new Pose(85.889, 85.143)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-137))
                .build();

        Gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.889, 85.143),
                                new Pose(109.797, 46.587),
                                new Pose(130.849, 57.677)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ReleaseGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.849, 57.677),
                                new Pose(118.530, 61.743),
                                new Pose(127.992, 67.631)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                .build();

        ShootRelease = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(127.992, 67.631),
                                new Pose(86.107, 84.854)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-137))
                .build();

        Stack1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.107, 84.854),
                                new Pose(83.900, 32.317),
                                new Pose(130.318, 35.110)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();


    }






        @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Turret(this);
        intake = new Intake(this);
        drive = new MecanumDrive(hardwareMap,telemetry);


        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.9);
        sleep(200);
        follower.update();
        buildPaths();


        SequenceAuto = new SequentialCommand(
                PedroToCommand(PreShoot, true),

                intake.intake(),
                new InstantCommand(()->flywheel.setTargetVelocity(shootPower)),
                new InstantCommand(()->flywheel.setHoodAngle(turretPitch)),
                flywheel.WaitForRPM(2000),
                new InstantCommand(flywheel::releaseShooter),
                new WaitCommand(500),
                new InstantCommand(flywheel::blockShooter),

                new InstantCommand(()->flywheel.setTargetVelocity(1000)),

                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(Stack3,true)
                ),
                PedroToCommand(Stack3Shoot,true),

                intake.intake(),
                new InstantCommand(()->flywheel.setTargetVelocity(shootPower)),
                new InstantCommand(()->flywheel.setHoodAngle(turretPitch)),
                flywheel.WaitForRPM(1500),
                new InstantCommand(flywheel::releaseShooter),
                new WaitCommand(500),
                new InstantCommand(flywheel::blockShooter),

                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(GateOpen,true)
                ),

                new WaitCommand(1500),
                PedroToCommand(GateShoot,true),

                intake.intake(),
                new InstantCommand(()->flywheel.setTargetVelocity(3100)),
                new InstantCommand(()->flywheel.setHoodAngle(0.15)),
                flywheel.WaitForRPM(2000),
                new InstantCommand(flywheel::releaseShooter),
                new WaitCommand(1000),
                new InstantCommand(flywheel::blockShooter),

                intake.stop(),
                new InstantCommand(flywheel::disable)

//                PedroToCommand(intakeCorner, false)
        );


        waitForStart();
        Command.run(new ParallelCommand(flywheel.reset(),intake.stop()));
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.update();

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        scheduler.schedule(SequenceAuto, flywheel.update());

        while(opModeIsActive()){

            for (LynxModule lynxModule : lynxModules) {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }

            scheduler.update();
            follower.update();
            panelsTelemetry.update();


            log("Actual Localizer Pose", follower.getPose().toString());
            log("Pose x", follower.getPose().getX());
            log("Pose y", follower.getPose().getY());
            log("Current heading", Math.toDegrees(follower.getPose().getHeading()));
            log("Turret Velocity", flywheel.getCurrentVelocity());
            log("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));

            telemetry.update();
        }
    }
}

