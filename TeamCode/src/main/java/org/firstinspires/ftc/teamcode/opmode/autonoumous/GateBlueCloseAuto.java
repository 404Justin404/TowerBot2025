package org.firstinspires.ftc.teamcode.opmode.autonoumous;

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
import org.firstinspires.ftc.teamcode.pedroPathing.SmartLocalizerPedro;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@Configurable
@Autonomous(name="Gate Blue Close Auto", group="Auto")
public class GateBlueCloseAuto extends LinearOpMode {

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
    private static  SequentialCommand SequenceAuto, SequenceShoot, SequenceGate;
    protected boolean isRed = false;
    private final Pose startPose = new Pose(25.045310853530054, 128.32876712328766, Math.toRadians(-37));
    private final double shootPower = 2200;
    private final double turretPitch = 0.05;

    public PathChain PreShoot;
    public PathChain Stack2;
    public PathChain Stack2Shoot;
    public PathChain Stack3;
    public PathChain GateIn,
            GateIntermediary,
                    GateOut;
    public PathChain Stack3Shoot;
    public PathChain Stack1;
    public PathChain Stack1Shoot;


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
                                new Pose(25.045, 128.329),
                                new Pose(58.428, 85.012)
                        )
                )
                .setVelocityConstraint(50)
                .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))
                .build();

        Stack2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.428, 85.012),
                                new Pose(48.854, 58.743),
                                new Pose(20, 57)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Stack2Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20, 57),
                                new Pose(58.428, 85.012)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-54))
                .build();

        GateIn = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.428, 85.012),
                                new Pose(48.852, 58.5),
                                new Pose(33, 58)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(-54), Math.toRadians(155))
                .build();

        GateIntermediary = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(33, 58),
                                new Pose(14.5, 58)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(155))
                .build();

        GateOut = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.5, 58),
                                new Pose(58.428, 85.012)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(-54))
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
        follower.update();
        buildPaths();

        SmartLocalizerPedro localizerPedro = (SmartLocalizerPedro) follower.getPoseTracker().getLocalizer();

        // Repetitive Command
        SequenceShoot = new SequentialCommand(
                intake.intake(),
                flywheel.velAuto(shootPower,turretPitch),
                flywheel.WaitForRPM(1000),
                new InstantCommand(flywheel::releaseShooter),
                new WaitCommand(550),
                new InstantCommand(flywheel::blockShooter),
                intake.slowIntake()
        );

        SequenceGate = new SequentialCommand(
                PedroToCommand(GateIn,true),
                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(GateIntermediary,true)
                ),
                new WaitCommand(1200),
                PedroToCommand(GateOut,true)
        );



        SequenceAuto = new SequentialCommand(
                PedroToCommand(PreShoot, true),
                SequenceShoot,

                new ParallelCommand(
                        intake.intake(),
                        PedroToCommand(Stack2,false)
                ),

                PedroToCommand(Stack2Shoot,true),
                SequenceShoot,

                SequenceGate, // FIRST GATE
                SequenceShoot,

                SequenceGate, // SECOND GATE
                SequenceShoot,

                SequenceGate, // SECOND GATE
                SequenceShoot,

                SequenceGate, // SECOND GATE
                SequenceShoot,

                intake.stop(),
                new InstantCommand(flywheel::disable)
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

            log("Intake voltage",intake.getCurrentAmps());
            intake.BallNumber(intake.getCurrentAmps());

           localizerPedro.getTelemetry(telemetry);
//            log("Actual Localizer Pose", follower.getPose().toString());
            log("Pose x", follower.getPose().getX());
            log("Pose y", follower.getPose().getY());
            log("Current heading", Math.toDegrees(follower.getPose().getHeading()));
            log("Turret Velocity", flywheel.getCurrentVelocity());
            log("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));

            telemetry.update();
        }
    }
}

